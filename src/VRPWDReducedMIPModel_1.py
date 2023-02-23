import gurobipy as gp
import networkx as nx

from gurobipy import GRB
from itertools import permutations
from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from utils import verbose_print, available_cpu_count


class VRPWDReducedMIPModel_1:
    def _create_solution(self, tour: list, drone_covered: dict) -> dict:
        truck_solution = [self.instance.dpd_nodes[j] for j in tour]
        node_covers = {
            self.instance.dpd_nodes[i]: [
                self.instance.dpd_nodes[j] for j in drone_covered[i]
            ]
            for i in drone_covered.keys()
        }
        solution = {"truck": [], "drone_1": [], "drone_2": []}
        # create a dictionnary with the demand of each node
        node_demands = {
            node: self.instance.graph.nodes[node]["demand"]
            for node in self.instance.dpd_nodes
        }
        for i, x in enumerate(truck_solution[:-1]):
            # assuming the drone delivery nodes are ordered by decreasing furthest distance
            # assign drone deliveries
            start_index = len(solution["truck"])
            if x in node_covers.keys():
                drone_availability = {1: 0, 2: 0}
                while len(node_covers[x]) > 0:
                    drone_to_use = min(drone_availability, key=drone_availability.get)
                    wait_time = drone_availability[drone_to_use]
                    if wait_time > 0:  # wait for drone to use
                        solution["truck"].append((x, x, wait_time + 0.001))
                    solution["truck"].append((x, x, 30, "d" + str(drone_to_use)))
                    for drone in drone_availability:
                        drone_availability[drone] = max(
                            0, drone_availability[drone] - wait_time - 30
                        )
                    z = node_covers[x].pop()
                    go_time = self.instance.drone_time_matrix[x - 1][z - 1]
                    solution["drone_" + str(drone_to_use)].append((x, z, go_time))
                    solution["drone_" + str(drone_to_use)].append((z, x, go_time))
                    drone_availability[drone_to_use] += 2 * go_time
                wait_time = max(drone_availability.values())
                if wait_time > 0:  # wait for drone to use
                    solution["truck"].append((x, x, wait_time + 0.001))

            # then we find the longest waiting time and assign truck delivery there
            if x in node_demands.keys() and node_demands[x] > 0.0:
                index = len(solution["truck"])
                value = 0
                for k in range(start_index, len(solution["truck"])):
                    if (
                        len(solution["truck"][k]) == 3
                        and solution["truck"][k][2] > value
                    ):
                        value = solution["truck"][k][2]
                        index = k
                if value == 0:
                    solution["truck"].append((x, x, 60, node_demands[x]))
                else:
                    solution["truck"][index] = (x, x, 60, node_demands[x])
                    if value > 60:
                        solution["truck"].insert(index + 1, (x, x, value - 60 + 0.001))

            # then we make our way to the next demand node
            y = truck_solution[i + 1]
            sp = nx.shortest_path(
                self.instance.graph, x, y, weight="travel_time", method="dijkstra"
            )
            for j, a in enumerate(sp[:-1]):
                b = sp[j + 1]
                transit_time = self.instance.graph.edges[a, b]["travel_time"]
                solution["truck"].append((a, b, transit_time))
        return solution

    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Reduced_MIP"
        self.model = gp.Model("VRPWDR")
        self.nodes = [0] + [
            i
            for i in range(1, len(instance.dpd_nodes))
            if instance.deposit != instance.dpd_nodes[i]
        ]
        self.road_time = {
            (i, j): instance.dpd_time_matrix[i][j]
            for i, j in permutations(self.nodes, 2)
        }
        self.demand = {
            i: instance.graph.nodes[instance.dpd_nodes[i]]["demand"] for i in self.nodes
        }
        self.drone_time = {
            (i, j): instance.drone_time_matrix[instance.dpd_nodes[i] - 1][
                instance.dpd_nodes[j] - 1
            ]
            for i, j in permutations(self.nodes, 2)
            if self.demand[j] > 0
        }
        self.max_demand = {(i, j): self.demand[j] for i, j in self.drone_time.keys()}

        pass

        # Variables:
        self.x = self.model.addVars(
            self.road_time.keys(), obj=self.road_time, vtype=GRB.BINARY, name="x"
        )
        self.y1 = self.model.addVars(
            self.drone_time.keys(), ub=self.max_demand, vtype=GRB.INTEGER, name="y1"
        )
        self.y2 = self.model.addVars(
            self.drone_time.keys(), ub=self.max_demand, vtype=GRB.INTEGER, name="y2"
        )
        self.T = self.model.addVars(self.nodes, obj=1.0, vtype=GRB.CONTINUOUS, name="T")

        # Flow Constraints:
        self.model.addConstrs(
            self.x.sum("*", j) == self.x.sum(j, "*") for j in self.nodes
        )
        self.model.addConstrs(self.x.sum("*", j) <= 1 for j in self.nodes)
        self.model.addConstr(self.x.sum(0, "*") == 1)
        # Demand Constraints:
        self.model.addConstrs(
            self.demand[j] * self.x.sum("*", j)
            + self.y1.sum("*", j)
            + self.y2.sum("*", j)
            == self.demand[j]
            for j in self.nodes
        )
        # Drone launch Constraints:
        self.model.addConstrs(
            self.y1[i, j] <= self.demand[j] * self.x.sum("*", i)
            for i, j in self.y1.keys()
        )
        self.model.addConstrs(
            self.y2[i, j] <= self.demand[j] * self.x.sum("*", i)
            for i, j in self.y2.keys()
        )
        # Delivery time Constraints:
        self.model.addConstrs(
            self.T[i]
            >= 30 * self.y1.sum(i, "*")
            + 2
            * gp.quicksum(
                self.drone_time[i, j] * self.y1[i, j]
                for j in self.nodes
                if j != i and self.demand[j] > 0
            )
            for i in self.nodes
        )
        self.model.addConstrs(
            self.T[i]
            >= 30 * self.y2.sum(i, "*")
            + 2
            * gp.quicksum(
                self.drone_time[i, j] * self.y2[i, j]
                for j in self.nodes
                if j != i and self.demand[j] > 0
            )
            for i in self.nodes
        )
        self.model.addConstrs(
            self.T[i]
            >= 30 * (self.y1.sum(i, "*") + self.y2.sum(i, "*"))
            + 60 * self.x.sum("*", i)
            for i in self.nodes
            if self.demand[i] > 0
        )

    def solve(
        self,
        time_limit: int = 3600,
        max_gap: float = 0.00001,
        nb_threads: int = available_cpu_count(),
    ) -> VRPWDSolution:
        verbose = self.instance._VERBOSE
        vprint = verbose_print(verbose)
        vprint(
            "==================== MIP CASE 1 *WITH NO RECHARGE AND NO DELIVERY TIME* RESOLUTION ===================="
        )
        self.model.Params.OutputFlag = int(verbose)
        self.model.Params.TimeLimit = time_limit
        self.model.Params.MIPGap = max_gap
        self.model.Params.Threads = nb_threads

        def _subtourelim(model, where):
            if where == GRB.Callback.MIPSOL:
                # make a list of edges selected in the solution
                vals = model.cbGetSolution(model._vars[0])
                selected = gp.tuplelist(
                    (i, j) for i, j in model._vars[0].keys() if vals[i, j] > 0.5
                )
                drone1_vals = model.cbGetSolution(model._vars[1])
                drone2_vals = model.cbGetSolution(model._vars[2])
                drone_covered = gp.tuplelist()
                drone_covered = gp.tuplelist(
                    (i, j)
                    for i, j in model._vars[1].keys()
                    if drone1_vals[i, j] > 0.5 or drone2_vals[i, j] > 0.5
                )
                # find the shortest soubtour in the selected edge list
                tour = _subtour(selected, drone_covered)
                if len(tour) < len(self.nodes):
                    # total demand of nodes that are not part of the tour
                    nt_demand = sum(
                        [self.demand[i] for i in self.nodes if i not in tour]
                    )
                    tour_routes = gp.tuplelist(
                        (i, j)
                        for i, j in model._vars[0].keys()
                        if i in tour and j in tour
                    )
                    for a, b in tour_routes:
                        if 0 in tour:
                            model.cbLazy(
                                nt_demand * model._vars[0][a, b]
                                <= nt_demand
                                * gp.quicksum(
                                    model._vars[0][i, j]
                                    for i, j in model._vars[0].keys()
                                    if i in tour and j not in tour
                                )
                                + gp.quicksum(
                                    model._vars[1][i, j]
                                    for i, j in model._vars[1].keys()
                                    if i in tour and j not in tour
                                )
                                + gp.quicksum(
                                    model._vars[2][i, j]
                                    for i, j in model._vars[2].keys()
                                    if i in tour and j not in tour
                                )
                            )
                        else:
                            model.cbLazy(
                                model._vars[0][a, b]
                                <= gp.quicksum(
                                    model._vars[0][i, j]
                                    for i, j in model._vars[0].keys()
                                    if i in tour and j not in tour
                                )
                            )

        # Given a tuplelist of edges, find the shortest subtour
        # Tour = all the nodes traveled and covered by the vehicle and its drones in a non disjoint path
        def _subtour(edges, drone_covered):
            unvisited = self.nodes[:]
            cycle = self.nodes[:]  # Dummy - guaranteed to be replaced
            while unvisited:  # true if list is non-empty
                thiscycle = []
                neighbors = unvisited
                while neighbors:
                    current = neighbors[0]
                    thiscycle.append(current)
                    unvisited.remove(current)
                    for _, j in drone_covered.select(current, "*"):
                        if j in unvisited:
                            thiscycle.append(j)
                            unvisited.remove(j)
                    for i, _ in drone_covered.select("*", current):
                        if i in unvisited:
                            thiscycle.append(i)
                            unvisited.remove(i)
                            current = i

                    neighbors = [
                        j for _, j in edges.select(current, "*") if j in unvisited
                    ]
                if len(thiscycle) <= len(cycle):
                    cycle = thiscycle  # New shortest subtour
                    return thiscycle
            return cycle

        def _final_truck_tour(edges):
            unvisited = self.nodes[:]
            drone_covered = {}
            tour = []
            neighbors = [self.nodes[0]]
            while neighbors:
                current = neighbors[0]
                tour.append(current)
                unvisited.remove(current)
                neighbors = [j for _, j in edges.select(current, "*") if j in unvisited]
            return tour

        self.model._vars = [self.x, self.y1, self.y2]
        self.model.Params.lazyConstraints = 1
        self.model.optimize(_subtourelim)

        # Create solution
        vals = self.model.getAttr("x", self.x)
        selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
        drone1_vals = self.model.getAttr("x", self.y1)
        drone2_vals = self.model.getAttr("x", self.y2)
        drone_vals = {
            (i, j): round(drone1_vals[i, j] + drone2_vals[i, j])
            for i, j in drone1_vals.keys()
            if round(drone1_vals[i, j] + drone2_vals[i, j]) > 0
        }

        truck_tour = _final_truck_tour(selected) + [0]

        drone_covered = {}
        for i, j in drone_vals.keys():
            if i not in drone_covered.keys():
                drone_covered[i] = []
            for _ in range(drone_vals[i, j]):
                drone_covered[i].append(j)
        for i in drone_covered.keys():

            def _drone_dist(j):
                return self.drone_time[i, j]

            drone_covered[i].sort(key=_drone_dist, reverse=True)
        solution = self._create_solution(truck_tour, drone_covered)

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
