import networkx as nx

from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from TSPGreedy import TSPGreedy
from TSPMIPModel import TSPMIPModel


class VRPWDHeuristic:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.init_sol = TSPMIPModel(self.instance).solve()

    def compute_time_savings(self):
        truck_route = self.init_sol.solution["truck"]
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
        # delete time savings that are composed of the same nodes in the tuple
        time_savings = [x for x in time_savings if x[0][0] != x[0][1]]
        # delete time savings that are composed of nodes that are not connected
        time_savings = [
            x for x in time_savings if self.instance.graph.has_edge(x[0][0], x[0][1])
        ]
        return time_savings

    def create_trucks_moves(self, time_savings, truck_route):
        moves = []
        for _tuple in time_savings:
            moves.append([_tuple[0][0], _tuple[0][1]])

        for i in range(len(moves)):
            for j in range(len(truck_route) - 1):
                if truck_route[j][0] == moves[i][0]:
                    moves[i].append(
                        truck_route[j - 1][2]
                        + self.instance.graph.edges[moves[i][0], moves[i][1]][
                            "travel_time"
                        ]
                    )

        for i in range(len(moves)):
            moves[i] = tuple(moves[i])

        return moves

    def solve(self):
        time_savings = self.compute_time_savings()
        truck_route = self.init_sol.solution["truck"]
        print(truck_route)
        new_truck_moves = self.create_trucks_moves(time_savings, truck_route)
        print(new_truck_moves)

        # drone_1_route = self.init_sol.solution["drone_1"]
        # drone_2_route = self.init_sol.solution["drone_2"]

        # for i in range(len(new_truck_moves)):
        #     for j in range(len(truck_route) - 1):
        #         if truck_route[j][0] == new_truck_moves[i]:
        #             # remove this move and insert the new one
        #             truck_route.remove(truck_route[j])
        #             truck_route.remove(truck_route[j + 1])
        #             truck_route.insert(
        #                 j,
        #                 new_truck_moves[i],
        #             )

        pass
