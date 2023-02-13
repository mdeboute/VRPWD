from VRPWDData import VRPWDData
from TSPMIPModel import TSPMIPModel
from utils import verbose_print
import time
import networkx as nx


class VRPWDHeuristic_2:
    def __init__(self, instance: VRPWDData, number_of_drones: int):
        self.instance = instance
        self.ordened_demands_nodes = []
        self.number_of_drones = number_of_drones
        self.init_sol = TSPMIPModel(self.instance).solve()

    def first_stage(self):
        """first stage part of the heuristic
        -given the TSP solution, affect the k drones to the k first nodes
        while the truck goes to the k+1th node etc"""
        print("deposit : ", self.instance.deposit)
        print("initial sol : ", self.init_sol)
        number_of_drones = 2
        number_of_vehicles = number_of_drones + 1
        dpd_nodes_per_vehicle = (
            {}
        )  # dict {0:[...], 1:[...], ..., k:[...]} with 0 is the truck dpd nodes list and the other drone dpd nodes list
        print("solution : ", self.init_sol.solution["truck"])
        print("dpd nodes : ", self.instance.dpd_nodes)
        for i in range(len(self.init_sol.solution["truck"]) - 1):
            if (
                self.init_sol.solution["truck"][i][1] in self.instance.dpd_nodes[1:]
                and self.init_sol.solution["truck"][i][1]
                not in self.ordened_demands_nodes
            ):
                self.ordened_demands_nodes.append(self.init_sol.solution["truck"][i][1])
        self.ordened_demands_nodes.append(self.instance.deposit)
        self.ordened_demands_nodes.insert(0, self.instance.deposit)
        print("ordened demand node : ", self.ordened_demands_nodes)
        for i in range(number_of_vehicles):
            dpd_nodes_per_vehicle[i] = []
            for node in self.ordened_demands_nodes:
                if self.ordened_demands_nodes.index(node) % number_of_vehicles == i:
                    dpd_nodes_per_vehicle[i].append(node)
        print(dpd_nodes_per_vehicle)

        return dpd_nodes_per_vehicle

    def second_stage(self, dpd_nodes_per_vehicle):
        """find the best node to launch drones within each sp between two truck visited demande nodes"""
        print("===============SECOND STAGE==============")
        # create dict to update the demandd
        solution = {str(i): [] for i in range(self.number_of_drones + 1)}
        # get each move that the truc will do in term of demand node
        pair_by_pair_truck_dpd_nodes = [
            (dpd_nodes_per_vehicle[0][i], dpd_nodes_per_vehicle[0][i + 1])
            for i in range(len(dpd_nodes_per_vehicle[0]) - 1)
        ]
        print("result = ", pair_by_pair_truck_dpd_nodes)
        # where to launch drones
        # loop over the sp
        # compute shortest path
        counter = 0
        for src_dest in pair_by_pair_truck_dpd_nodes:
            print(
                "index du node {} : {}".format(
                    src_dest[0], self.instance.dpd_nodes.index(src_dest[0])
                )
            )
            print(
                "index du node {} : {}".format(
                    src_dest[1], self.instance.dpd_nodes.index(src_dest[1])
                )
            )
            self.instance.dpd_time_matrix[self.instance.dpd_nodes.index(src_dest[0])][
                self.instance.dpd_nodes.index(src_dest[0])
            ]
            sp_truck = nx.shortest_path(
                self.instance.graph, src_dest[0], src_dest[1], weight="travel_time"
            )
            sp_truck_tt = nx.shortest_path_length(
                self.instance.graph, src_dest[0], src_dest[1], weight="travel_time"
            )
            print("pcc entre les deux : ", sp_truck)
            print("with a tt : {} s".format(sp_truck_tt))
            # find the launching node for each drone
            min_d_tt = [1000000000 for i in range(1, self.number_of_drones + 1)]
            min_d_node = [None for i in range(1, self.number_of_drones + 1)]
            d_back_to_truck_time = [None for i in range(1, self.number_of_drones + 1)]
            print("min_d_tt : ", min_d_tt)
            for key in range(1, self.number_of_drones + 1):
                print("key = ", key)
                print(
                    "drone {} target : {}".format(
                        key, dpd_nodes_per_vehicle[key][counter]
                    )
                )
                for current_truck_node in sp_truck:
                    print("current_truck_node : ", current_truck_node)
                    sp_d = self.instance.drone_time_matrix[current_truck_node - 1][
                        dpd_nodes_per_vehicle[key][counter] - 1
                    ]
                    print("sp_d = ", sp_d)
                    if sp_d < min_d_tt[key - 1]:
                        print("better")
                        min_d_tt[key - 1] = sp_d
                        min_d_node[key - 1] = current_truck_node
                d_back_to_truck_time[key - 1] = self.instance.drone_time_matrix[
                    min_d_node[key - 1] - 1
                ][sp_truck[-1]]
            # here the best nodes are founded
            for o, a, b in zip(
                [i for i in range(1, self.number_of_drones + 1)], min_d_node, min_d_tt
            ):
                print("best d{} node : {} with {} s".format(o, a, b))
            print("min_d_tt : ", min_d_tt)
            print("min_d_node", min_d_node)
            print("d_back_to_truck_time : ", d_back_to_truck_time)
            # create solution
            self.create_solution(
                solution,
                sp_truck,
                min_d_node,
                min_d_tt,
                d_back_to_truck_time,
                dpd_nodes_per_vehicle,
                counter,
            )

    def create_solution(
        self,
        solution,
        sp_truck,
        min_d_node,
        min_d_tt,
        d_back_to_truck_time,
        dpd_nodes_per_vehicle,
        counter,
    ):
        """create the solution for truck and drone, for current sp truck move"""
        print("===========CREATE SOLUTION ==========")
        preparing_drone_time = 30
        delivering_truck_time = 60
        truck_destination = sp_truck[-1]
        print("truck destination : ", truck_destination)
        demand_dict = {}
        for node in self.instance.graph.nodes:
            if self.instance.graph.nodes[node]["demand"] >= 1:
                demand_dict[node] = self.instance.graph.nodes[node]["demand"]
        print("demand_dict : ", demand_dict)
        # compute reaching time to common destination
        drone_reaching_cdest_time = [
            round(a + b, 3) for a, b in zip(min_d_tt, d_back_to_truck_time)
        ]
        print("drone_reaching_cdest_time : ", drone_reaching_cdest_time)
        # is truck at destination ie demand node ? ->deliver demand
        # by def, the truck only deliver the truck destination node, so if it travel another demand node, it wont deliver it
        for current_truck_node in sp_truck:
            print("current_truck_node :", current_truck_node)
            indices = []  # list in which current node
            for i, item in enumerate(min_d_node):
                print("i : ", i + 1)
                print("item : ", item)
                if item == current_truck_node:
                    indices.append(i + 1)
            print("indices : ", indices)
            # if drone to launch from this node
            nb_drones_to_launch = len(indices)
            if nb_drones_to_launch != 0:
                print("--------LAUNCHING NODE---------")
                for i in indices:
                    if len(solution["0"]) == 0:
                        starting_node = self.instance.deposit
                    else:
                        starting_node = solution["0"][-1][1]
                    # create the move truck event
                    tt = nx.shortest_path_length(
                        self.instance.graph,
                        starting_node,
                        current_truck_node,
                        weight="travel_time",
                    )
                    truck_move_event = (starting_node, current_truck_node, tt)
                    if tt != 0:
                        print("move truck event : ", truck_move_event)
                        solution["0"].append(truck_move_event)
                    # create the event preparing drone event
                    preparing_event = (
                        current_truck_node,
                        current_truck_node,
                        preparing_drone_time,
                        "d{}".format(i),
                    )
                    print("preparing_event : ", preparing_event)
                    # add it to the truck solution
                    solution["0"].append(preparing_event)
                    # create drone move to reach the dpd node
                    drone_move1_event = (
                        current_truck_node,
                        dpd_nodes_per_vehicle[i][counter],
                        min_d_tt[i - 1],
                    )
                    # create drone move to come back to the truck
                    drone_move2_event = (
                        dpd_nodes_per_vehicle[i][counter],
                        truck_destination,
                        d_back_to_truck_time[i - 1],
                    )
                    solution[str(i)].append(drone_move1_event)
                    solution[str(i)].append(drone_move2_event)
                    # print('solution[{}] : {}'.format(i,solution[str(i)]))
                    # update the demand
                    demand_dict[current_truck_node] = 0
                    # does the drone need to wait for truck ?
            else:
                if current_truck_node == truck_destination:
                    print("------TRUCK ON DESTINATION NODE------")
                    print("demand : ", demand_dict[current_truck_node])
                    time_to_deliver_demand = (
                        delivering_truck_time * demand_dict[current_truck_node]
                    )
                    print("time_to_deliver_demand = ", time_to_deliver_demand)
                    if len(solution["0"]) == 0:
                        starting_node = self.instance.deposit
                    else:
                        starting_node = solution["0"][-1][1]
                    # create the truck move event
                    tt = nx.shortest_path_length(
                        self.instance.graph,
                        starting_node,
                        current_truck_node,
                        weight="travel_time",
                    )
                    move_truck_event = (starting_node, current_truck_node, tt)
                    # add it to the truck solution
                    if tt != 0:
                        solution["0"].append(move_truck_event)
                    # create the delivery event
                    delivery_event = (
                        current_truck_node,
                        current_truck_node,
                        time_to_deliver_demand,
                        demand_dict[current_truck_node],
                    )
                    tt_last_launching_to_dest_plus_deliver = (
                        solution["0"][-1][2] + time_to_deliver_demand
                    )
                    # add it to the truck solution
                    solution["0"].append(delivery_event)
                    print("solution 0 :", solution["0"])
                    # update demand in the dict
                    demand_dict[current_truck_node] = 0
                    # does the truck need to wait for drone ?
                    max_drone_time = max(drone_reaching_cdest_time)
                    print(
                        "tt_last_launching_to_dest_plus_deliver = ",
                        tt_last_launching_to_dest_plus_deliver,
                    )
                    print("max_drone_time = ", max_drone_time)
                    # real_tt_truck_to_dest=nx.shortest_path_length(self.instance.graph,sp_truck[0],sp_truck[-1],weight='travel_time')+self.number_of_drones*preparing_drone_time
                    if max_drone_time > tt_last_launching_to_dest_plus_deliver:
                        waiting_time = (
                            max_drone_time - tt_last_launching_to_dest_plus_deliver
                        )
                        waiting_truck_event = (
                            sp_truck[-1],
                            sp_truck[-1],
                            round(waiting_time, 3),
                        )
                        solution["0"].append(waiting_truck_event)
                    elif max_drone_time < tt_last_launching_to_dest_plus_deliver:
                        for time in drone_reaching_cdest_time:
                            if time < tt_last_launching_to_dest_plus_deliver:
                                waiting_drone_time = (
                                    tt_last_launching_to_dest_plus_deliver - time
                                )
                                waiting_drone_event = (
                                    current_truck_node,
                                    current_truck_node,
                                    waiting_drone_time,
                                )
                                solution[
                                    str(drone_reaching_cdest_time.index(time) + 1)
                                ].append(waiting_drone_event)
        print(solution)

    def solve(self):
        start_time = time.time()
        dpd_nodes_per_vehicle = self.first_stage()
        self.second_stage(dpd_nodes_per_vehicle)
        end_time = time.time()
        processing_time = end_time - start_time
        verbose_print("processing time : {} s".format(processing_time))
