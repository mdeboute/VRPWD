import time
import gurobipy as gp
import networkx as nx

from VRPWDData import VRPWDData
from TSPMIPModel import TSPMIPModel
from VRPWDSolution import VRPWDSolution
from utils import verbose_print
from gurobipy import GRB


class VRPWDPathHeuristic_2:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Path_Heuristic"
        tsp_solution = TSPMIPModel(self.instance).solve()
        self.init_sol = tsp_solution.solution
        self.init_runtime = tsp_solution.runtime
        # First and last tuples of truck are (depot, depot, 0, 0) to signal start and stop. Useful for heuristic
        self.init_sol["truck"].append((instance.deposit, instance.deposit, 0.0, 0.0))
        if self.init_sol["truck"][0][0] != self.init_sol["truck"][0][1]:
            self.init_sol["truck"].insert(
                0, (instance.deposit, instance.deposit, 0.0, 0.0)
            )

    def create_solution(self, selected_paths, gained_time):
        solution = {"truck": [], "drone_1": [], "drone_2": []}
        last_drone_used = 2
        init_truck = self.init_sol["truck"]
        t = 0
        for p in selected_paths.keys():
            path = selected_paths[p]
            start_node = path["path"][0][0]
            end_node = path["path"][-1][0]
            while t < path["start_index"]:
                solution["truck"].append(init_truck[t])
                t += 1
            drone_costs = {1: 0, 2: 0}
            drone_to_use = last_drone_used % 2 + 1
            for d in range(path["drones_used"]):
                d_node = path["d_nodes"][d]
                solution["truck"].append(
                    (start_node, start_node, 30, "d" + str(drone_to_use))
                )
                go_time = self.instance.drone_time_matrix[start_node - 1][d_node - 1]
                solution["drone_" + str(drone_to_use)].append(
                    (start_node, d_node, go_time)
                )
                back_time = self.instance.drone_time_matrix[d_node - 1][end_node - 1]
                solution["drone_" + str(drone_to_use)].append(
                    (d_node, end_node, back_time)
                )
                drone_costs[drone_to_use] = 30 * (d + 1) + go_time + back_time
                drone_to_use = drone_to_use % 2 + 1
            last_drone_used = max(drone_costs, key=drone_costs.get)

            solution["truck"].append(
                init_truck[path["start_index"]]
            )  # t == path["start_index"]
            sp = nx.shortest_path(
                self.instance.graph,
                start_node,
                end_node,
                weight="travel_time",
                method="dijkstra",
            )
            # create final solution
            for j, a in enumerate(sp[:-1]):
                b = sp[j + 1]
                time = self.instance.graph.edges[a, b]["travel_time"]
                solution["truck"].append((a, b, time))
            wait_time = (
                path["new_cost"] - path["truck_cost"] + gained_time[p]
            )  # gained_time is negative, thus reduced wait time
            # possibly improve the solution further by delivering end node during wait_time if wait_time > end_node truck delivery time
            if wait_time > 0:
                solution["truck"].append((end_node, end_node, wait_time))
            t = path["end_index"]
        while t < len(init_truck):
            solution["truck"].append(init_truck[t])
            t += 1
        for tup in solution["truck"]:
            if tup[2] == 0.0:
                # print(tup)
                solution["truck"].remove(tup)
        return solution

    def init_sol_demand_paths(self, demand_limit=2):
        truck = self.init_sol["truck"]
        complete_paths = []
        incomplete_paths = [[truck[0]]]
        inc_pth_cml_demand = [
            0
        ]  # total sustified demand on given path, excluding the demand nodes at the extremities of the path
        path_start_index = [0]
        for i in range(1, len(truck)):
            step = truck[i]
            if step[0] == step[1]:  # truck delivery step
                for p in range(len(incomplete_paths) - 1, -1, -1):
                    if (
                        inc_pth_cml_demand[p] + step[3] > demand_limit
                        or i == len(truck) - 1
                    ):
                        if inc_pth_cml_demand[p] > 0:
                            incomplete_paths[p].append(step)

                            path_cost = self.calculate_drone_path_cover_cost(
                                incomplete_paths[p]
                            )
                            if (
                                path_cost[2] < 0
                            ):  # true if cost is improved by using drones on path
                                path_info = {
                                    "path": incomplete_paths[p],
                                    "start_index": path_start_index[p],
                                    "end_index": i,
                                    "d_nodes": path_cost[0],
                                    "new_cost": path_cost[1],
                                    "gain": path_cost[2],
                                    "drones_used": path_cost[3],
                                    "truck_cost": path_cost[4],
                                    "drone1_cost": path_cost[5],
                                    "drone2_cost": path_cost[6],
                                }

                                complete_paths.append(path_info)
                        incomplete_paths.pop(p)
                        inc_pth_cml_demand.pop(p)
                        path_start_index.pop(p)
                    else:
                        incomplete_paths[p].append(step)
                        inc_pth_cml_demand[p] += self.instance.graph.nodes[step[0]][
                            "demand"
                        ]
                incomplete_paths.append([step])
                inc_pth_cml_demand.append(0)
                path_start_index.append(i)
            else:
                for path in incomplete_paths:
                    path.append(step)
        return complete_paths

    def calculate_drone_path_cover_cost(
        self, path
    ):  # assuming a maximum of 2 drone deliveries can be done on this path
        old_path_cost = sum(arc[2] for arc in path)
        start_node = path[0][0]
        end_node = path[-1][1]
        d_nodes = []
        for i in range(1, len(path) - 2):
            if path[i][0] == path[i][1]:
                demand = path[i][3]
                for j in range(int(demand)):
                    d_nodes.append(path[i][0])
        drone1_cost = (
            30
            + self.instance.drone_time_matrix[start_node - 1][d_nodes[0] - 1]
            + self.instance.drone_time_matrix[d_nodes[0] - 1][end_node - 1]
        )
        nb_drones_used = 1
        drone2_cost = 0
        if len(d_nodes) > 1:
            drone2_cost = (
                30
                + self.instance.drone_time_matrix[start_node - 1][d_nodes[1] - 1]
                + self.instance.drone_time_matrix[d_nodes[1] - 1][end_node - 1]
            )
            nb_drones_used = 2
            if drone2_cost < drone1_cost:
                drone2_cost = drone2_cost + 30
            else:
                d_nodes.reverse()
                drone1_cost = drone1_cost + 30
        start_end_travel_time = self.instance.dpd_time_matrix[
            self.instance.dpd_nodes.index(start_node)
        ][self.instance.dpd_nodes.index(end_node)]
        truck_cost = 30 * nb_drones_used + path[0][2] + start_end_travel_time

        new_path_cost = max(truck_cost, drone1_cost, drone2_cost)
        print("Old path cost =", old_path_cost, ", New path cost =", new_path_cost)
        return (
            d_nodes,
            new_path_cost,
            new_path_cost - old_path_cost,
            nb_drones_used,
            truck_cost,
            drone1_cost,
            drone2_cost,
        )

    def calculate_path_overlap_matrix(self, paths):
        ovlp_matrix = [[False for _ in range(len(paths))] for _ in range(len(paths))]
        for i in range(len(paths)):
            for j in range(i):
                start1 = paths[i]["start_index"]
                end1 = paths[i]["end_index"]
                start2 = paths[j]["start_index"]
                end2 = paths[j]["end_index"]
                if max(paths[i]["drones_used"], paths[j]["drones_used"]) == 2:
                    ovlp_matrix[i][j] = ovlp_matrix[j][i] = (
                        (start2 < start1 < end2)
                        or (start2 < end1 < end2)
                        or (start1 < start2 < end1)
                        or (start1 < end2 < end1)
                    )
        return ovlp_matrix

    def calculate_interpath_time_gain_matrix(self, paths):
        iptg_matrix = [[0 for _ in range(i + 1)] for i in range(len(paths))]
        truck = self.init_sol["truck"]

        for i in range(len(paths)):
            for j in range(i + 1):
                curr_path = paths[i]
                past_path = paths[j]
                possible_gain = past_path["new_cost"] - past_path["truck_cost"]
                arrival_window = truck[past_path["end_index"]][2]
                iptg_matrix[i][j] = -min(possible_gain, arrival_window)
                if (
                    past_path["new_cost"] != past_path["truck_cost"]
                    and past_path["end_index"] == curr_path["start_index"]
                ):
                    sup_drone_time = max(
                        past_path["drone1_cost"], past_path["drone2_cost"]
                    )
                    inf_drone_time = min(
                        past_path["drone1_cost"], past_path["drone2_cost"]
                    )

                    possible_gain = sup_drone_time - max(
                        past_path["truck_cost"], inf_drone_time
                    )
                    arrival_window = 30 * (curr_path["drones_used"] - 1)
                    iptg_matrix[i][j] = -min(possible_gain, arrival_window)
        return iptg_matrix

    def solve(
        self,
        time_limit: int = 600,
        max_gap: float = 0.00001,
        nb_threads: int = 4,
    ):
        start_preprocess_time = time.time()
        paths_info = self.init_sol_demand_paths()
        overlap_matrix = self.calculate_path_overlap_matrix(paths_info)
        inter_path_time_gain_matrix = self.calculate_interpath_time_gain_matrix(
            paths_info
        )
        preprocess_time = time.time() - start_preprocess_time

        model = gp.Model("Minimum Paths Selection Problem")
        n = len(paths_info)
        paths = [i for i in range(n)]
        gain_time = {i: paths_info[i]["gain"] for i in paths}
        ipg_time = {
            (i, j): inter_path_time_gain_matrix[i][j]
            for i in paths
            for j in range(i + 1)
        }

        x = model.addVars(gain_time.keys(), obj=gain_time, vtype=GRB.BINARY, name="x")
        y = model.addVars(ipg_time.keys(), obj=0.0, vtype=GRB.BINARY, name="y")
        lb = {i: ipg_time[i, i] for i in paths}
        z = model.addVars(paths, lb=lb, ub=0.0, obj=1.0, vtype=GRB.CONTINUOUS, name="z")

        model.addConstrs(
            x[i] + x[j] <= 1 for i in paths for j in range(i) if overlap_matrix[i][j]
        )
        model.addConstrs(
            2 * x[i] + 2 * x[j] <= 2 * y[i, j] + 2 for (i, j) in ipg_time.keys()
        )
        model.addConstrs(2 * y[i, j] <= x[i] + x[j] for (i, j) in ipg_time.keys())
        model.addConstrs(
            z[j]
            >= ipg_time[i, j] * y[i, j]
            - (1 - y[i, j]) * (paths_info[j]["new_cost"] - paths_info[j]["truck_cost"])
            for (i, j) in ipg_time.keys()
        )

        model.Params.OutputFlag = int(self.instance._VERBOSE)
        model.Params.TimeLimit = time_limit
        model.Params.MIPGap = max_gap
        model.Params.Threads = nb_threads

        model.optimize()

        # Create solution
        x_vals = model.getAttr("x", x)
        y_vals = model.getAttr("x", y)
        z_vals = model.getAttr("x", z)

        selected_paths = {i: paths_info[i] for i in paths if x_vals[i] > 0.5}
        gained_time = {i: z_vals[i] for i in paths if x_vals[i] > 0.5}

        solution = self.create_solution(selected_paths, gained_time)
        objective_value = sum(
            solution["truck"][i][2] for i in range(len(solution["truck"]))
        )
        runtime = self.init_runtime + preprocess_time + model.Runtime
        gap = model.MIPGap * 100

        if model.Status == GRB.OPTIMAL:
            return VRPWDSolution(
                instance=self.instance,
                algorithm=self.__algorithm,
                objective_value=round(objective_value),
                runtime=runtime,
                gap=gap,
                solution=solution,
                verbose=self.instance._VERBOSE,
            )
        elif model.Status == GRB.FEASIBLE:
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
