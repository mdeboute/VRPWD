import networkx as nx

from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from TSPGreedy import TSPGreedy
from TSPMIPModel import TSPMIPModel


class VRPWDHeuristic_1:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.init_sol = TSPMIPModel(self.instance).solve()

    def compute_time_savings(self):
        truck_route = self.init_sol.solution["truck"]
        demands_nodes = self.instance.dpd_nodes[1:]
        time_savings = {}
        for i in range(len(truck_route) - 1):
            if truck_route[i][1] in demands_nodes:
                time_truck_move_1 = truck_route[i][-1]
                time_truck_move_2 = truck_route[i + 1][-1]
                time_drones_move = (
                    2
                    * self.instance.drone_time_matrix[truck_route[i][0]][
                        truck_route[i][1]
                    ]
                    * self.instance.graph.nodes[truck_route[i][1]]["demand"]
                )
                if truck_route[i][0] == truck_route[i + 1][1]:
                    time_truck_move_3 = 0
                else:
                    time_truck_move_3 = nx.shortest_path_length(
                        self.instance.graph,
                        truck_route[i][0],
                        truck_route[i + 1][1],
                        weight="travel_time",
                        method="dijkstra",
                    )
                delta = (time_truck_move_1 + time_truck_move_2) - (
                    time_drones_move + time_truck_move_3
                )
                if delta > 0:
                    time_savings[(truck_route[i][0], truck_route[i + 1][1])] = delta
        # sort time savings
        time_savings = sorted(time_savings.items(), key=lambda x: x[1], reverse=True)
        return time_savings

    def create_trucks_moves(self, time_savings, truck_route):
        moves = []
        for _tuple in time_savings:
            moves.append([_tuple[0][0], _tuple[0][1]])

        for i in range(len(moves)):
            for j in range(len(truck_route) - 1):
                if truck_route[j][0] == moves[i][0]:
                    # TODO: finish
                    pass

        for i in range(len(moves)):
            moves[i] = tuple(moves[i])

        return moves

    def solve(self):
        time_savings = self.compute_time_savings()
        # sum of time savings
        sum_time_savings = 0
        for _tuple in time_savings:
            sum_time_savings += _tuple[1]
        print("Sum of time savings:", sum_time_savings)
        # compute the percentage of time savings with the objective value
        percentage = (sum_time_savings / self.init_sol.objective_value) * 100
        print(f"So a decrease of {percentage:.2f}%!")

        # truck_route = self.init_sol.solution["truck"]
        # print(truck_route)
        # new_truck_moves = self.create_trucks_moves(time_savings, truck_route)
        # print(new_truck_moves)

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
        # TODO: finish
