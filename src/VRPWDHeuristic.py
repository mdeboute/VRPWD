import networkx as nx

from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from TSPGreedy import TSPGreedy


class VRPWDHeuristic:
    def __init__(self, instance: VRPWDData):
        self.instance = instance

    def solve(self):
        init_sol = TSPGreedy(self.instance).solve()

        truck_route = init_sol.solution["truck"]
        drone_1_route = init_sol.solution["drone_1"]
        drone_2_route = init_sol.solution["drone_2"]

        demands_nodes = self.instance.dpd_nodes[1:]

        time_savings = {}

        for i in range(len(truck_route) - 1):
            if truck_route[i][1] in demands_nodes:
                amount_demand = self.instance.graph.nodes[truck_route[i][1]]["demand"]
                time_truck_move_1 = (
                    self.instance.graph.edges[truck_route[i][0], truck_route[i][1]][
                        "travel_time"
                    ]
                    + 60 * amount_demand
                )
                time_truck_move_2 = self.instance.graph.edges[
                    truck_route[i + 1][0], truck_route[i + 1][1]
                ]["travel_time"]
                time_drones_move = (
                    2
                    * self.instance.drone_time_matrix[truck_route[i][0]][
                        truck_route[i][1]
                    ]
                    * amount_demand
                )
                time_truck_move_3 = nx.shortest_path_length(
                    self.instance.graph,
                    truck_route[i][0],
                    truck_route[i + 1][1],
                    weight="travel_time",
                    method="dijkstra",
                )
                time_savings[(truck_route[i][0], truck_route[i + 1][1])] = (
                    time_truck_move_1 + time_truck_move_2
                ) - (time_drones_move + time_truck_move_3)
        # sort time savings
        time_savings = sorted(time_savings.items(), key=lambda x: x[1], reverse=True)
        # delete negative time savings
        time_savings = [x for x in time_savings if x[1] > 0]
        # delete time savings with tuple that are composed by the same node
        time_savings = [x for x in time_savings if x[0][0] != x[0][1]]

        print(time_savings)
        print(truck_route)

        pass
