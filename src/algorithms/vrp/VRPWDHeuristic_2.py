import time
import networkx as nx

from core.VRPWDData import VRPWDData
from algorithms.tsp.TSPMIPModel import TSPMIPModel
from core.VRPWDSolution import VRPWDSolution
from core.utils import verbose_print


class VRPWDHeuristic_2:
    def __init__(self, instance: VRPWDData, number_of_drones: int, policy: str):
        self.instance = instance
        self._available_policies = ["drone_inter", "truck_inter", "mix"]
        self.policy = policy
        if self.policy not in self._available_policies:
            raise Exception("ERROR: this policy is not available")
        self.__algorithm = "Super_Node_Greedy"
        self.ordoned_demands_nodes = []
        self.number_of_drones = number_of_drones
        self.init_sol = TSPMIPModel(self.instance).solve()

        global vprint
        vprint = verbose_print(self.instance._VERBOSE)

    def first_stage(self, tour=None):
        """Create the vehicule-node affectation dictionary from a tour if given or from OPT(TSP) otherwise"""

        super_nodes_dict = {}
        demand_dict = {}
        for node in self.instance.graph.nodes:
            if self.instance.graph.nodes[node]["demand"] >= 1:
                demand_dict[node] = self.instance.graph.nodes[node]["demand"]
            if self.instance.graph.nodes[node]["demand"] > 1:
                super_nodes_dict[node] = self.instance.graph.nodes[node]["demand"]
        self.super_nodes_dict = super_nodes_dict
        if tour == None:
            for i in range(len(self.init_sol.solution["truck"]) - 1):
                if (
                    self.init_sol.solution["truck"][i][1] in self.instance.dpd_nodes[1:]
                    and self.init_sol.solution["truck"][i][1]
                    not in self.ordoned_demands_nodes
                ):
                    self.ordoned_demands_nodes.append(
                        self.init_sol.solution["truck"][i][1]
                    )
            # in case of deposit is a demand node
            if self.instance.deposit not in self.ordoned_demands_nodes:
                self.ordoned_demands_nodes.insert(0, self.instance.deposit)
            self.ordoned_demands_nodes.append(self.instance.deposit)
        else:
            self.ordoned_demands_nodes = tour
        ordoned_super_nodes = [
            k for k in self.ordoned_demands_nodes if k in super_nodes_dict.keys()
        ]
        self.ordoned_super_nodes = ordoned_super_nodes
        node_vehicle_dict = {}
        begin = 0
        for node in self.ordoned_demands_nodes:
            if node in self.ordoned_super_nodes or node == self.instance.deposit:
                node_vehicle_dict[node] = 0
                begin = 1
            else:
                node_vehicle_dict[node] = begin
                begin += 1
                if begin == 3:
                    begin = 0
        self.node_vehicle_dict = node_vehicle_dict
        return node_vehicle_dict

    def second_stage(self, node_vehicle_dict):
        """Create the solution"""

        # create dict to update the demandd
        solution = {"truck": []}
        for i in range(1, self.number_of_drones + 1):
            solution.setdefault("drone_{}".format(i), [])
        demand_dict = {}
        for node in self.instance.graph.nodes:
            if self.instance.graph.nodes[node]["demand"] >= 1:
                demand_dict[node] = self.instance.graph.nodes[node]["demand"]
        truck_pair_dpd_nodes = []
        for node in self.ordoned_demands_nodes:
            node_vehicle = node_vehicle_dict[node]
            if node_vehicle == 0:
                truck_pair_dpd_nodes.append(node)
                if len(truck_pair_dpd_nodes) == 2:
                    sp_truck = nx.shortest_path(
                        self.instance.graph,
                        truck_pair_dpd_nodes[0],
                        truck_pair_dpd_nodes[-1],
                        weight="travel_time",
                    )
                    entire_dpd_nodes = [
                        x
                        for x in self.ordoned_demands_nodes
                        if self.ordoned_demands_nodes.index(x)
                        >= self.ordoned_demands_nodes.index(truck_pair_dpd_nodes[0])
                        and self.ordoned_demands_nodes.index(x)
                        <= self.ordoned_demands_nodes.index(truck_pair_dpd_nodes[1])
                    ]
                    if (
                        node != self.instance.deposit
                        and entire_dpd_nodes[-1] == self.instance.deposit
                    ):
                        entire_dpd_nodes.pop(-1)
                    # we arrive at the end list --> entire dpd node is the starting truck node until the end of the list
                    if node == self.instance.deposit:
                        entire_dpd_nodes = self.ordoned_demands_nodes[
                            self.ordoned_demands_nodes.index(truck_pair_dpd_nodes[0]) :
                        ]

                    drone_target = [None for i in range(self.number_of_drones)]
                    go_tt_list = [None for i in range(self.number_of_drones)]
                    back_tt_list = [None for i in range(self.number_of_drones)]
                    # if there is drone to launch
                    if len(entire_dpd_nodes) > 2:
                        for node in entire_dpd_nodes:
                            if node not in truck_pair_dpd_nodes:
                                node_vehicle = node_vehicle_dict[node]
                                drone_target[node_vehicle - 1] = node
                                # compute travel time
                                go = self.instance.drone_time_matrix[
                                    truck_pair_dpd_nodes[0] - 1
                                ][node - 1]
                                go_tt_list[node_vehicle - 1] = go
                                back = self.instance.drone_time_matrix[node - 1][
                                    truck_pair_dpd_nodes[1] - 1
                                ]
                                back_tt_list[node_vehicle - 1] = back
                    self.create_solution(
                        solution,
                        sp_truck,
                        drone_target,
                        go_tt_list,
                        back_tt_list,
                        demand_dict,
                    )
                    truck_pair_dpd_nodes = [truck_pair_dpd_nodes[-1]]
        is_feasible = all(v == 0 for v in demand_dict.values())
        return (solution, is_feasible)

    def create_solution(
        self,
        solution,
        sp_truck,
        drone_target_list,
        go_tt_list,
        back_tt_list,
        demand_dict,
    ):
        """Create the solution for truck and drone, for current sp truck move"""

        preparing_drone_time = 30
        delivering_truck_time = 60
        launched_drones = [False for i in range(self.number_of_drones)]
        truck_chrono = [0 for i in range(1, self.number_of_drones + 1)]
        truck_destination = sp_truck[-1]

        # if there is drone to launch
        if not all(t == None for t in drone_target_list):
            # check if there is a drone demand node over the truck sp -> the reaction depends of the policy
            for node in sp_truck:
                if node in drone_target_list:
                    drone_number = drone_target_list.index(node) + 1
                    if self.policy == "truck_inter":
                        pass
                    elif self.policy == "mix":
                        pass
            # drone which are not launched yet (ie drone inter policy and other)
            for d in launched_drones:
                if not d and drone_target_list[launched_drones.index(d)] != None:
                    drone_number = launched_drones.index(d) + 1
                    # create truck launch event
                    lauching_drone_event = (
                        sp_truck[0],
                        sp_truck[0],
                        preparing_drone_time,
                        "d{}".format(drone_number),
                    )
                    solution["truck"].append(lauching_drone_event)
                    truck_chrono = [
                        truck_chrono[i] + preparing_drone_time
                        if launched_drones[i]
                        else truck_chrono[i]
                        for i in range(self.number_of_drones)
                    ]
                    launched_drones[drone_number - 1] = True
                    # create drone go move
                    drone_go_move = (
                        sp_truck[0],
                        drone_target_list[drone_number - 1],
                        go_tt_list[drone_number - 1],
                    )
                    solution["drone_{}".format(drone_number)].append(drone_go_move)
                    # update demand
                    demand_dict[drone_target_list[drone_number - 1]] -= 1
                    # create drone back to truck move
                    drone_back_move = (
                        drone_target_list[drone_number - 1],
                        truck_destination,
                        back_tt_list[drone_number - 1],
                    )
                    solution["drone_{}".format(drone_number)].append(drone_back_move)
        # drone are launched or there is no drone to launch -> truck move toward its destination
        for node in sp_truck:
            if len(solution["truck"]) == 0:
                starting_node = self.instance.deposit
            else:
                starting_node = solution["truck"][-1][1]
            # create move event
            tt = round(
                nx.shortest_path_length(
                    self.instance.graph, starting_node, node, weight="travel_time"
                ),
                3,
            )
            if tt != 0:
                move_event = (starting_node, node, tt)
                solution["truck"].append(move_event)
                truck_chrono = [
                    truck_chrono[i] + tt if launched_drones[i] else truck_chrono[i]
                    for i in range(self.number_of_drones)
                ]
            if node == truck_destination:
                if node in demand_dict.keys():
                    # create delivery event
                    delivery_event = (node, node, delivering_truck_time)
                    truck_chrono = [
                        truck_chrono[i] + delivering_truck_time
                        if launched_drones[i]
                        else truck_chrono[i]
                        for i in range(self.number_of_drones)
                    ]
                    solution["truck"].append(delivery_event)
                    # update demand
                    demand_dict[node] = 0
                # deal with waiting time
                # split drone arrival into 2 list : before truck arrival and after truck arrival
                if not all(t == None for t in drone_target_list):
                    before_truck = []
                    after_truck = []
                    truck_chrono_for_compared = []
                    drone_tt = [
                        a + b
                        for a, b in zip(go_tt_list, back_tt_list)
                        if a != None and b != None
                    ]
                    for time_t, time_d in zip(truck_chrono, drone_tt):
                        if time_t != None and time_d != None and time_t <= time_d:
                            after_truck.append(time_d)
                            truck_chrono_for_compared.append(time_t)
                        elif time_t != None and time_d != None and time_t > time_d:
                            before_truck.append(time_d)
                    # case 1 --> drones wait for the truck
                    if len(before_truck) != 0:
                        # compute waiting time for each drone
                        for time in before_truck:
                            drone_index = drone_tt.index(time) + 1
                            d_waiting_time = truck_chrono[drone_index - 1] - time
                            if d_waiting_time != 0:
                                if d_waiting_time < 0:
                                    raise Exception("ERROR: drone waiting time < 0")
                            else:
                                waiting_drone_event = (
                                    truck_destination,
                                    truck_destination,
                                    round(d_waiting_time, 3),
                                )
                                # add it to the solution
                                solution["drone_{}".format(drone_index)].append(
                                    waiting_drone_event
                                )
                    # case 2 --> truck waits for drones
                    if len(after_truck) != 0:
                        truck_waiting_time = max(
                            [
                                d - t
                                for d, t in zip(after_truck, truck_chrono_for_compared)
                            ]
                        )
                        if truck_waiting_time < 0:
                            raise Exception("ERROR: drone waiting time < 0")

                        else:
                            waiting_truck_event = (
                                truck_destination,
                                truck_destination,
                                round(truck_waiting_time, 3),
                            )
                            solution["truck"].append(waiting_truck_event)

    def _improve_affectation(self, solution, node_vehicle_dict):
        """Improve the affectation node-vehicle for a given tour"""

        vprint("Improving affectation...")
        start_time = time.time()
        result = []
        start = 0
        for i, val in enumerate(self.ordoned_demands_nodes):
            if val in self.ordoned_super_nodes:
                result.append(self.ordoned_demands_nodes[start : i + 1])
                start = i
        result = [sublist for sublist in result if len(sublist) > 1]
        # exchange the drone target
        # Copy the original dictionary so as not to modify it
        new_node_vehicle_dict = node_vehicle_dict.copy()
        best_obj = self._calculate_obj_value(solution)
        is_improved = False
        # Iterate through all key-value pairs in the dictionary
        for i, (k, v) in enumerate(node_vehicle_dict.items()):
            # If the value is 1 and the next value is 2, or vice versa
            if i == len(node_vehicle_dict) - 1:
                break
            if (v == 1 and list(node_vehicle_dict.values())[i + 1] == 2) or (
                v == 2 and list(node_vehicle_dict.values())[i + 1] == 1
            ):
                # Swap key values for both nodes
                (
                    new_node_vehicle_dict[k],
                    new_node_vehicle_dict[list(node_vehicle_dict.keys())[i + 1]],
                ) = (
                    new_node_vehicle_dict[list(node_vehicle_dict.keys())[i + 1]],
                    new_node_vehicle_dict[k],
                )
                # create the solution from the dictionary
                new_sol, is_feasible = self.second_stage(new_node_vehicle_dict)
                # Calculate the cost of the new solution
                new_obj = self._calculate_obj_value(new_sol)
                if new_obj < best_obj and is_feasible:
                    node_vehicle_dict = new_node_vehicle_dict.copy()
                    vprint("improvement founded")
                    best_obj = new_obj
                    best_sol = new_sol
                    best_node_vehicle_dict = node_vehicle_dict
                    is_improved = True
                # Otherwise, go back to the previous solution
                else:
                    (
                        new_node_vehicle_dict[k],
                        new_node_vehicle_dict[list(node_vehicle_dict.keys())[i + 1]],
                    ) = (
                        new_node_vehicle_dict[list(node_vehicle_dict.keys())[i + 1]],
                        new_node_vehicle_dict[k],
                    )
        if not is_improved:
            best_sol = solution
            best_obj = self._calculate_obj_value(solution)
            best_node_vehicle_dict = node_vehicle_dict
        # Return the modified dictionary or the original dictionary if no modification has been made
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time:", processing_time)
        return (best_sol, True, best_node_vehicle_dict, is_improved)

    def _calculate_obj_value(self, solution):
        return sum([move[2] for move in solution["truck"]])

    def solve(self) -> VRPWDSolution:
        vprint("============== SOLVE ==============")
        start_time = time.time()
        node_vehicle_dict = self.first_stage()
        solution, is_feasible = self.second_stage(node_vehicle_dict)
        (
            solution,
            is_feasible,
            best_node_vehicle_dict,
            is_improved,
        ) = self._improve_affectation(solution, self.node_vehicle_dict)
        vprint("is final sol feasible? :", is_feasible)
        if is_feasible:
            objective_value = self._calculate_obj_value(solution)
            vprint("final_solution:", solution)
            vprint("final_objectif_value:", objective_value)
            vprint("init_node_vehicle_dict:", self.node_vehicle_dict)
            vprint("final_node_vehicle_dict:", best_node_vehicle_dict)
            vprint("init has been improved? :", is_improved)
            end_time = time.time()
            processing_time = end_time - start_time

            vprint(f"Processing_time: {processing_time}sec")
            vprint(f"Initial objective value: {self.init_sol.objective_value}")
            vprint(f"New objective value: {objective_value}")
            vprint(
                f"So a decrease of: {(self.init_sol.objective_value - objective_value) / self.init_sol.objective_value * 100:.2f}%"
            )

            return VRPWDSolution(
                instance=self.instance,
                algorithm=self.__algorithm,
                objective_value=objective_value,
                runtime=processing_time,
                gap="unknown",
                solution=solution,
                verbose=self.instance._VERBOSE,
            )
