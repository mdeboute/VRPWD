import gurobipy as gp
from gurobipy import GRB
from itertools import combinations
from TSPWDData import TSPWDData
from TSPWDSolution import TSPWDSolution


class TSPMIPModel:
    def __init__(self, instance: TSPWDData):
        self.instance = instance
        self.__algorithm = "MIP"
        self.model = gp.Model("TSP")
        self.n = len(instance.dpd_time_matrix)
        self.nodes = [i for i in range(len(instance.dpd_time_matrix))]
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
        verbose: bool = False,
        time_limit: int = 600,
        max_gap: float = 0.00001,
        nb_threads: int = 4,
    ) -> TSPWDSolution:
        self.model.Params.OutputFlag = int(verbose)
        self.model.Params.TimeLimit = time_limit
        self.model.Params.MIPGap = max_gap
        self.model.Params.Threads = nb_threads

        def subtourelim(model, where):
            if where == GRB.Callback.MIPSOL:
                # make a list of edges selected in the solution
                vals = model.cbGetSolution(model._vars)
                selected = gp.tuplelist(
                    (i, j) for i, j in model._vars.keys() if vals[i, j] > 0.5
                )
                # find the shortest cycle in the selected edge list
                tour = subtour(selected)
                if len(tour) < len(self.nodes):
                    # add subtour elimination constr. for every pair of cities in subtour
                    model.cbLazy(
                        gp.quicksum(model._vars[i, j] for i, j in combinations(tour, 2))
                        <= len(tour) - 1
                    )

        # Given a tuplelist of edges, find the shortest subtour
        def subtour(edges):
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
        self.model.optimize(subtourelim)

        # Create solution
        vals = self.model.getAttr("x", self.x)
        selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
        tour = subtour(selected) + [0]

        solution = [self.instance.dpd_nodes[j] for j in tour]

        _runtime = self.model.Runtime

        # Get solution
        if self.model.Status == GRB.OPTIMAL:
            print(
                f"Optimal Result: runtime={_runtime:.2f}sec; objective={int(self.model.ObjVal)}; gap={self.model.MIPGap:.4f}%"
            )
            return TSPWDSolution(
                self.instance,
                self.__algorithm,
                int(self.model.ObjVal),
                solution,
                self.instance._VERBOSE,
            )
        elif self.model.Status == GRB.FEASIBLE:
            print(
                f"Result: runtime={_runtime:.2f}sec; objective={int(self.model.ObjVal)}; gap={100*self.model.MIPGap:.4f}%"
            )
            return TSPWDSolution(
                self.instance,
                self.__algorithm,
                int(self.model.ObjVal),
                solution,
                self.instance._VERBOSE,
            )
        else:
            print(f"No solution found in {time_limit} seconds!")

    def __str__(self):
        return f" TSPWDMIPModel(instance={self.instance})"

    def __repr__(self):
        return self.__str__()
