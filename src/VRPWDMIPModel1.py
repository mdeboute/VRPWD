import gurobipy as gp
from gurobipy import GRB
import numpy as np
from itertools import permutations
from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from utils import verbose_print


class VRPWDMIPModel1:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Gurobi"
        self.model = gp.Model("VRPWD1")
        self.n = len(instance.graph.nodes)
        self.nodes = [node for node in instance.graph.nodes()]
        self.demand_nodes = instance.dpd_nodes
        self.deposit = instance.deposit
        self.road_time = {(a, b): instance.graph.edges[a, b]['travel_time'] 
                          for e in instance.graph.edges() 
                          for a,b in permutations([e[0], e[1]], 2)}
        self.drone_time = {(i, j): instance.drone_matrix[i-1][j-1] for i, j in permutations(self.nodes, 2) if j in self.demand_nodes}
        self.demand = {node : instance.graph.nodes[node]["demand"] for node in self.nodes}
        self.max_demand = max(self.demand)
        pass

        # Variables:
        self.x = self.model.addVars(self.road_time.keys(), obj=self.road_time, vtype=GRB.BINARY, name='x')
        self.y1 = self.model.addVars(self.drone_time.keys(), ub=self.max_demand, vtype=GRB.INTEGER, name='y1')
        self.y2 = self.model.addVars(self.drone_time.keys(), ub=self.max_demand, vtype=GRB.INTEGER, name='y2')
        self.T = self.model.addVars(self.nodes, obj=1.0, vtype=GRB.CONTINUOUS, name='T')

        # Flow Constraints:
        self.model.addConstrs(self.x.sum(j, "*") == self.x.sum("*", j) for j in self.nodes)
        # Demand Constraints:
        self.model.addConstrs(self.demand[j] * self.x.sum("*", j) + self.y1.sum("*", j) + self.y2.sum("*", j) >= self.demand[j] for j in self.demand_nodes)
        self.model.addConstr(self.x.sum(self.deposit, "*") >= 1)
        #self.model.addConstrs(self.demand[j] * self.x.sum(j, "*") >= self.demand[j] for j in self.nodes)
        # Drone launch Constraints:
        self.model.addConstrs(self.y1[i,j] <= self.demand[j] * self.x.sum("*", i) for i,j in self.y1.keys())
        self.model.addConstrs(self.y2[i,j] <= self.demand[j] * self.x.sum("*", i) for i,j in self.y2.keys())
        # Drone delivery time Constraints:
        self.model.addConstrs(self.T[i] >= 2 * gp.quicksum(self.drone_time[i,j] * self.y1[i,j] for j in self.demand_nodes if j!=i) for i in self.nodes)
        self.model.addConstrs(self.T[i] >= 2 * gp.quicksum(self.drone_time[i,j] * self.y2[i,j] for j in self.demand_nodes if j!=i) for i in self.nodes)

    def solve(
        self,
        time_limit: int = 600,
        max_gap: float = 0.00001,
        nb_threads: int = 4,
    ) -> VRPWDSolution:
        verbose = self.instance._VERBOSE
        vprint = verbose_print(verbose)
        vprint("==================== MIP CASE 1 *WITH NO RECHARGE AND NO DELIVERY TIME* RESOLUTION ====================")
        self.model.Params.OutputFlag = int(verbose)
        self.model.Params.TimeLimit = time_limit
        self.model.Params.MIPGap = max_gap
        self.model.Params.Threads = nb_threads

        def subtourelim(model, where):
            if where == GRB.Callback.MIPSOL:
                # make a list of edges selected in the solution
                vals = model.cbGetSolution(model._vars[0])
                selected = gp.tuplelist(
                    (i, j) for i, j in model._vars[0].keys() if vals[i, j] > 0.5
                )
                drone1_vals = model.cbGetSolution(model._vars[1])
                drone2_vals = model.cbGetSolution(model._vars[2])
                drone_covered = [(i,j) for i,j in model._vars[1].keys() if drone1_vals[i, j]> 0.5 or drone2_vals[i, j] > 0.5]
                # find the shortest soubtour in the selected edge list
                tour = subtour(selected, drone_covered)
                if not is_tour_complete(tour):
                    # total demand of nodes that are not part of the tour
                    nt_demand = sum([self.demand[i] for i in self.demand_nodes if i not in tour])
                    # add subtour elimination constr. for every road selected in subtour
                    for a,b in model._vars[0].keys():
                        if a in tour and b in tour:
                            model.cbLazy(
                                nt_demand * model._vars[0][a,b] <= 
                                nt_demand * gp.quicksum(model._vars[0][i, j] for i,j in model._vars[0].keys() if i in tour and j not in tour) + 
                                gp.quicksum(model._vars[1][i, j] for i, j in model._vars[1].keys() if i in tour and j not in tour) + 
                                gp.quicksum(model._vars[2][i, j] for i, j in model._vars[2].keys() if i in tour and j not in tour)
                            )

        def is_tour_complete(tour):
            for node in self.demand_nodes:
                if node not in tour:
                    return False
            return True

        # Given a tuplelist of edges, find the shortest subtour
        # Tour = all the nodes traveled and covered by the vehicle and its drones in a non disjoint path
        def subtour(edges, drone_covered):
            mustvisit = self.demand_nodes[:]
            cycle = self.nodes[:]  # Dummy - guaranteed to be replaced
            while mustvisit:  # true if list is non-empty
                thiscycle = []
                visited = []
                tovisit = [mustvisit[0]]
                while tovisit:
                    #current node = i
                    current = tovisit.pop(0)
                    thiscycle.append(current)
                    visited.append(current)
                    if current in mustvisit:
                        mustvisit.remove(current)
                    for (i,j) in drone_covered:
                        if i==current:
                            thiscycle.append(j)
                            visited.append(j)
                            mustvisit.remove(j)
                        if j==current:
                            thiscycle.append(i)
                            visited.append(i)
                            if i in mustvisit:
                                mustvisit.remove(i)
                            current = i
                    neighbors = [j for _, j in edges.select(current, "*") if j not in visited]
                    for neighbor in neighbors:
                        tovisit.append(neighbor)
                if len(thiscycle) <= len(cycle):
                    cycle = thiscycle  # New shortest subtour
            return cycle

        def subtour_displayer(edges, drone_covered):
            mustvisit = self.demand_nodes[:]
            cycle = self.nodes[:]  # Dummy - guaranteed to be replaced
            while mustvisit:  # true if list is non-empty
                thiscycle = []
                visited = []
                tovisit = [mustvisit[0]]
                while tovisit:
                    #current node = i
                    current = tovisit.pop(0)
                    thiscycle.append(current)
                    visited.append(current)
                    if current in mustvisit:
                        mustvisit.remove(current)
                    for (i,j) in drone_covered:
                        if i==current:
                            thiscycle.append(j)
                            visited.append(j)
                            mustvisit.remove(j)
                        elif j==current:
                            thiscycle.append(i)
                            visited.append(i)
                            if i in mustvisit:
                                mustvisit.remove(i)
                            current = i
                    neighbors = [j for _, j in edges.select(current, "*") if j not in visited]
                    for neighbor in neighbors:
                        tovisit.append(neighbor)
                print(thiscycle)
                if len(thiscycle) <= len(cycle):
                    cycle = thiscycle  # New shortest subtour
            return cycle

        self.model._vars = [self.x, self.y1, self.y2]
        #self.model._vars = [self.x]
        #max_nb_lazy = len(self.x.keys()) - 2
        self.model.Params.lazyConstraints = 1
        self.model.optimize(subtourelim)
        #self.model.optimize()

        # Create solution
        vals = self.model.getAttr("x", self.x)
        selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
        drone1_vals = self.model.getAttr("x", self.y1)
        drone2_vals = self.model.getAttr("x", self.y2)
        drone_covered = [(i,j) for i,j in drone1_vals.keys() if drone1_vals[i, j]> 0.5 or drone2_vals[i, j] > 0.5]
        tour = subtour_displayer(selected, drone_covered)
        print("vehicle_route:", selected)
        print("drone_covered:", drone_covered)
        #print(tour)
        #print("demand(243) =", self.demand[243])
        solution = tour

        _runtime = self.model.Runtime

        # Get solution
        if self.model.Status == GRB.OPTIMAL:
            print(
                f"Optimal Result: runtime={_runtime:.2f}sec; objective={int(self.model.ObjVal)}; gap={self.model.MIPGap:.4f}%"
            )
            return """VRPWDSolution(
                self.instance,
                self.__algorithm,
                int(self.model.ObjVal),
                solution,
                self.instance._VERBOSE,
            )"""
        elif self.model.Status == GRB.FEASIBLE:
            print(
                f"Result: runtime={_runtime:.2f}sec; objective={int(self.model.ObjVal)}; gap={100*self.model.MIPGap:.4f}%"
            )
            return """VRPWDSolution(
                self.instance,
                self.__algorithm,
                int(self.model.ObjVal),
                solution,
                self.instance._VERBOSE,
            )"""
        else:
            print(f"No solution found in {time_limit} seconds!")

    def __str__(self):
        return f" VRPWDMIPModel1(instance={self.instance})"

    def __repr__(self):
        return self.__str__()
