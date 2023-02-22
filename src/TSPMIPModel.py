import gurobipy as gp
import networkx as nx

from gurobipy import GRB
from itertools import combinations
from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from utils import available_cpu_count


def create_solution(instance: VRPWDData, tour: list) -> dict:
    truck_solution = [instance.dpd_nodes[j] for j in tour]
    solution = {"truck": [], "drone_1": [], "drone_2": []}
    # create a dictionnary with the demand of each node
    node_demands = {
        node: instance.graph.nodes[node]["demand"] for node in instance.dpd_nodes
    }
    for i, x in enumerate(truck_solution[:-1]):
        if x in node_demands.keys() and node_demands[x] > 0.0:
            solution["truck"].append((x, x, 60, node_demands[x]))
        y = truck_solution[i + 1]
        sp = nx.shortest_path(
            instance.graph, x, y, weight="travel_time", method="dijkstra"
        )
        # create final solution
        for j, a in enumerate(sp[:-1]):
            b = sp[j + 1]
            time = instance.graph.edges[a, b]["travel_time"]
            solution["truck"].append((a, b, time))
    return solution


class TSPMIPModel:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "MIP"
        self.model = gp.Model("TSP")
        self.n = len(instance.dpd_time_matrix)
        self.nodes = [0] + [
            i
            for i in range(1, len(instance.dpd_nodes))
            if instance.deposit != instance.dpd_nodes[i]
        ]
        self.time = {
            (i, j): instance.dpd_time_matrix[i][j]
            for i, j in combinations(self.nodes, 2)
        }

        # Variables: is city 'i' adjacent to city 'j' on the tour?
        self.x = self.model.addVars(
            self.time.keys(), obj=self.time, vtype=GRB.BINARY, name="x"
        )

        # Symmetric direction: Copy the object
        for i, j in self.x.keys():
            self.x[j, i] = self.x[i, j]  # edge in opposite direction

        # Constraints: two edges incident to each city
        self.model.addConstrs(self.x.sum(c, "*") == 2 for c in self.nodes)

    def solve(
        self,
        time_limit: int = 3600,
        max_gap: float = 0.00001,
        nb_threads: int = available_cpu_count(),
    ) -> VRPWDSolution:
        self.model.Params.OutputFlag = int(self.instance._VERBOSE)
        self.model.Params.TimeLimit = time_limit
        self.model.Params.MIPGap = max_gap
        self.model.Params.Threads = nb_threads

        def _subtourelim(model, where):
            if where == GRB.Callback.MIPSOL:
                # make a list of edges selected in the solution
                vals = model.cbGetSolution(model._vars)
                selected = gp.tuplelist(
                    (i, j) for i, j in model._vars.keys() if vals[i, j] > 0.5
                )
                # find the shortest cycle in the selected edge list
                tour = _subtour(selected)
                if len(tour) < len(self.nodes):
                    # add subtour elimination constr. for every pair of cities in subtour
                    model.cbLazy(
                        gp.quicksum(model._vars[i, j] for i, j in combinations(tour, 2))
                        <= len(tour) - 1
                    )

        # Given a tuplelist of edges, find the shortest subtour
        def _subtour(edges):
            unvisited = self.nodes[:]
            cycle = self.nodes[:]  # Dummy - guaranteed to be replaced
            while unvisited:  # true if list is non-empty
                thiscycle = []
                neighbors = unvisited
                while neighbors:
                    current = neighbors[0]
                    thiscycle.append(current)
                    unvisited.remove(current)
                    neighbors = [
                        j for _, j in edges.select(current, "*") if j in unvisited
                    ]
                if len(thiscycle) <= len(cycle):
                    cycle = thiscycle  # New shortest subtour
            return cycle

        self.model._vars = self.x
        self.model.Params.lazyConstraints = 1
        self.model.optimize(_subtourelim)

        # Create solution
        vals = self.model.getAttr("x", self.x)
        selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
        tour = _subtour(selected) + [0]
        solution = create_solution(self.instance, tour)
        # to compute the objective value of the solution we have to sum the travel times
        objective_value = sum(
            solution["truck"][i][2] for i in range(len(solution["truck"]))
        )
        runtime = self.model.Runtime
        gap = self.model.MIPGap * 100
        if self.model.Status == GRB.OPTIMAL:
            return VRPWDSolution(
                instance=self.instance,
                algorithm=self.__algorithm,
                objective_value=round(objective_value),
                runtime=runtime,
                gap=gap,
                solution=solution,
                verbose=self.instance._VERBOSE,
            )
        elif not self.model.Status == GRB.INF_OR_UNBD:
            return VRPWDSolution(
                instance=self.instance,
                algorithm=self.__algorithm,
                objective_value=round(objective_value),
                runtime=runtime,
                gap=gap,
                solution=solution,
                verbose=self.instance._VERBOSE,
            )
        else:
            print(f"No solution found in {time_limit} seconds!")
