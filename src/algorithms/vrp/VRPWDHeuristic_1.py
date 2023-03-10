import networkx as nx
import time

from core.VRPWDData import VRPWDData
from core.VRPWDSolution import VRPWDSolution
from algorithms.tsp.TSPMIPModel import TSPMIPModel
from core.utils import verbose_print


class VRPWDHeuristic_1:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Basic_Greedy"
        self.init_sol = TSPMIPModel(self.instance).solve()
        self.demands_nodes = {
            node: int(self.instance.graph.nodes[node]["demand"])
            for node in self.instance.dpd_nodes[1:]
        }
        self.graph = self.instance.graph.copy()

        global vprint
        vprint = verbose_print(self.instance._VERBOSE)

    def _compute_time_savings(self):
        truck_route = self.init_sol.solution["truck"]
        time_savings = []
        for i in range(len(truck_route) - 2):
            if (
                len(truck_route[i]) == 3
                and truck_route[i][1] in self.demands_nodes
                and len(truck_route[i + 1]) == 4
            ):
                time_truck_move_1 = truck_route[i][-1]
                time_truck_deliver = truck_route[i + 1][-2]
                time_truck_move_2 = truck_route[i + 2][-1]
                time_drones_moves = self.demands_nodes[truck_route[i][1]] * (
                    2
                    * self.instance.drone_time_matrix[truck_route[i][0]][
                        truck_route[i][1]
                    ]
                    + 30
                )
                if truck_route[i][0] == truck_route[i + 2][1]:
                    time_truck_move_3 = 0
                else:
                    try:
                        # exclude the demand node from the path
                        self.graph.remove_node(truck_route[i][1])
                        time_truck_move_3 = nx.shortest_path_length(
                            self.graph,
                            truck_route[i][0],
                            truck_route[i + 2][1],
                            weight="travel_time",
                            method="dijkstra",
                        )
                    # add the demand node back to the graph
                    except nx.NetworkXNoPath:
                        time_truck_move_3 = float("inf")
                    finally:
                        self.graph.add_node(
                            truck_route[i][1],
                            demand=self.demands_nodes[truck_route[i][1]],
                        )
                delta = (time_truck_move_1 + time_truck_deliver + time_truck_move_2) - (
                    time_drones_moves + time_truck_move_3
                )
                if delta > 0:
                    time_savings.append(
                        (
                            truck_route[i][0],
                            truck_route[i][1],
                            truck_route[i + 2][1],
                            delta,
                        )
                    )

        time_savings.sort(key=lambda x: x[3], reverse=True)

        dst_nodes = []
        for _tuple in time_savings:
            dst_nodes.append(_tuple[1])
            if _tuple[0] in dst_nodes or _tuple[2] in dst_nodes:
                time_savings.remove(_tuple)
            if dst_nodes.count(_tuple[1]) > 1:
                # remove the tuple with the lowest delta
                time_savings.remove(
                    min(
                        time_savings,
                        key=lambda x: x[3] if x[1] == _tuple[1] else float("inf"),
                    )
                )

        # if the demand node have a demand > 2, we need to remove the tuple
        for _tuple in time_savings:
            if self.demands_nodes[_tuple[1]] > 2:
                time_savings.remove(_tuple)

        vprint(f"Time savings: {time_savings}\n")

        return time_savings

    def _create_new_moves(self, time_savings):
        truck_route = self.init_sol.solution["truck"]
        new_truck_route = []
        drone_1_route = []
        drone_2_route = []

        # moves to change are the moves that will be changed by the drones
        # take the two first elements of the time savings tuples
        moves_to_change = [tuple[0:2] for tuple in time_savings]
        demands_nodes_delivered_by_drones = [tuple[1] for tuple in time_savings]

        for i in range(len(truck_route)):
            src = truck_route[i][0]
            dst = truck_route[i][1]
            if (src, dst) in moves_to_change and len(truck_route[i + 1]) == 4:
                new_dest = time_savings[moves_to_change.index((src, dst))][2]
                drone_time_travel = self.instance.drone_time_matrix[src][dst]
                time_to_wait = (2 * drone_time_travel) + 0.001
                if self.demands_nodes[dst] == 1:
                    new_truck_route.append((src, src, 30, "d1"))
                    drone_1_route.append((src, dst, drone_time_travel))
                    drone_1_route.append((dst, src, drone_time_travel))
                    new_truck_route.append((src, src, time_to_wait))
                elif self.demands_nodes[dst] == 2:
                    new_truck_route.append((src, src, 30, "d1"))
                    new_truck_route.append((src, src, 30, "d2"))
                    drone_1_route.append((src, dst, drone_time_travel))
                    drone_1_route.append((dst, src, drone_time_travel))
                    drone_2_route.append((src, dst, drone_time_travel))
                    drone_2_route.append((dst, src, drone_time_travel))
                    new_truck_route.append((src, src, time_to_wait))
                if src != dst:
                    new_route = nx.shortest_path(
                        self.instance.graph,
                        src,
                        new_dest,
                        weight="travel_time",
                        method="dijkstra",
                    )
                    for j in range(len(new_route) - 1):
                        new_truck_route.append(
                            (
                                new_route[j],
                                new_route[j + 1],
                                self.instance.graph.edges[
                                    new_route[j], new_route[j + 1]
                                ]["travel_time"],
                            )
                        )
            # if one node of the move (the src or the dst) is delivered by a drone and the move is not mandatory to reach the destination, we pass
            elif (src in demands_nodes_delivered_by_drones) and len(
                truck_route[i - 1]
            ) == 4:
                pass
            elif (dst in demands_nodes_delivered_by_drones) and len(
                truck_route[i + 1]
            ) == 4:
                pass
            elif (src == dst) and ((src or dst) in demands_nodes_delivered_by_drones):
                pass
            else:
                new_truck_route.append(truck_route[i])

        return new_truck_route, drone_1_route, drone_2_route

    def solve(self) -> VRPWDSolution:
        vprint(f"Initial solution: {self.init_sol.solution}\n")
        vprint(f"Initial objective value: {self.init_sol.objective_value}\n")

        start_time = time.time()

        time_savings = self._compute_time_savings()

        new_truck_route, drone_1_route, drone_2_route = self._create_new_moves(
            time_savings
        )

        end_time = time.time()
        runtime = end_time - start_time + self.init_sol.runtime

        solution = {
            "truck": new_truck_route,
            "drone_1": drone_1_route,
            "drone_2": drone_2_route,
        }

        # to compute the objective value we just have to sum the travel times of the truck
        # i.e. the 3rd element of each tuple in the truck route
        objective_value = sum([move[2] for move in new_truck_route])

        vprint(f"New solution: {solution}\n")
        vprint(f"New objective value: {objective_value:.2f}\n")

        # so a decrease in % of the objective value is:
        vprint(
            f"So a decrease of: {(self.init_sol.objective_value - objective_value) / self.init_sol.objective_value * 100:.2f}%\n"
        )

        return VRPWDSolution(
            instance=self.instance,
            algorithm=self.__algorithm,
            objective_value=objective_value,
            runtime=runtime,
            gap="unknown",
            solution=solution,
            verbose=self.instance._VERBOSE,
        )
