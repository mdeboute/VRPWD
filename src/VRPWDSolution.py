import networkx as nx
import matplotlib.pyplot as plt
import time

from pathlib import Path
from utils import verbose_print
from VRPWDData import VRPWDData
from pathlib import Path


class VRPWDSolution:
    __BASE_DIR = Path(__file__).resolve().parent.parent

    def __init__(
        self,
        instance: VRPWDData,
        algorithm: str,
        objective_value: int,
        run_time: float,
        solution: dict,
        verbose: bool,
    ):
        self.__VERBOSE = verbose
        global vprint
        vprint = verbose_print(self.__VERBOSE)

        self.instance = instance
        self.algorithm = algorithm
        self.objective_value = objective_value
        self.run_time = run_time
        self.solution = solution
        self.graph = self._create_graph()

        self.__SOLUTION_DIR = (
            str(self.__BASE_DIR) + "/solution/" + self.instance._INSTANCE_NAME + "/"
        )

    def __str__(self):
        return f"Solution(objective_value={self.objective_value})"

    def __repr__(self):
        return self.__str__()

    def _create_graph(self):
        """Create the graph format of the solution"""

        vprint("=================== CREATE GRAPH SOLUTION ===================")
        start_time = time.time()
        # get solutions for each type  of vehicle
        truck_tour = self.solution["truck"]
        number_of_drones=2
        drone_1_tour = self.solution["drone_1"]
        drone_2_tour = self.solution["drone_2"]
        # create graph
        graph = nx.DiGraph()
        # add demands nodes
        for node in self.instance.dpd_nodes[1:]:
            demand_value = self.instance.graph.nodes[node]["demand"]
            coords = self.instance.graph.nodes[node]["coordinates"]
            inversed_coords = (coords[1], coords[0])  # another networkx curiosity 0_0??
            graph.add_node(
                node, coordinates=inversed_coords, deposit=False, demand=demand_value
            )
        # add deposit node
        graph_coords_deposit = self.instance.graph.nodes[self.instance.deposit][
            "coordinates"
        ]
        inversed_graph_coords_deposit = (
            graph_coords_deposit[1],
            graph_coords_deposit[0],
        )
        graph.add_node(
            self.instance.deposit,
            coordinates=inversed_graph_coords_deposit,
            deposit=True,
            demand=0,
        )
        # any case -> truck tour
        for move in truck_tour:
            #check for a mouvement tuple
            if len(move)==3 and move[0]!=move[1]:
                src = move[0]
                dest = move[1]
                tt = move[2]
                # check if the node already exists
                if not graph.has_node(dest):
                    # by construction, this node is not the deposit nor a demand node
                    # create the node
                    dest_coords = self.instance.graph.nodes[dest]["coordinates"]
                    inversed_dest_coords = (dest_coords[1], dest_coords[0])
                    graph.add_node(
                        dest, coordinates=inversed_dest_coords, deposit=False, demand=0
                    )
                graph.add_edge(src, dest, travel_time=tt, vehicle='truck')
        # case 1,2,3 -> add drone edges
        if self.instance._CASE > 0:
            for i in range(number_of_drones):
                drone_tour=self.solution['drone_{}'.format(i+1)]
                for move in drone_tour:
                    #check for a drone move
                    if len(move)==3 and move[0]!=move[1]:
                        src = move[0]
                        dest = move[1]
                        tt = move[2]
                        graph.add_edge(src, dest, travel_time=tt, vehicle='drone_{}'.format(i+1))
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("graph:", graph)
        vprint("processing time:", processing_time)
        return graph

    def plot(self):
        """Plot the graph"""

        vprint("==================== PLOT GRAPH ====================")
        # Draw graph
        coordinates = nx.get_node_attributes(self.graph, "coordinates")
        node_colors = []
        for node in self.graph.nodes():
            if self.graph.nodes[node]["deposit"]:
                node_colors.append("g")
            elif self.graph.nodes[node]["demand"] > 0:
                node_colors.append("r")
            else:
                node_colors.append("b")
        nx.draw(
            self.graph,
            coordinates,
            node_color=node_colors,
            with_labels=False,
            node_size=50,
            width=0.5,
        )
        # Show plot
        plt.show()

    def check(self):
        if self.instance._CASE == 0:
            # check if all demand nodes are in the tour
            for node in self.instance.dpd_nodes[1:]:
                if node not in [_tuple[0] for _tuple in list(self.solution["truck"])]:
                    print(f"ERROR: demand node {node} is not in the tour!")
                    return False
            # check that we start at the deposit
            if self.solution["truck"][0][0] != self.instance.deposit:
                print("ERROR: tour does not start at the deposit!")
                return False
            # check that we end at the deposit
            if self.solution["truck"][-1][1] != self.instance.deposit:
                print("ERROR: tour does not end at the deposit!")
                return False
            # check that we do not visit the deposit twice
            if [_tuple[0] for _tuple in list(self.solution["truck"])].count(
                self.instance.deposit
            ) > 2:
                print("ERROR: tour visits the deposit twice!")
                return False
        else:
            print("ERROR: checks for that case are not implemented yet!")
        return True

    def write(self):
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)
        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"

        truck_route = list(self.solution["truck"])
        drone_routes = {1: list(self.solution["drone_1"]), 2:list(self.solution["drone_2"])}
        coords = nx.get_node_attributes(self.graph, "coordinates")

        d_ix = {1: 0, 2: 0}
        drone_events = {}
        with open(_sol_file, "w") as f:
            f.write("TEMPS ; EVENEMENT ; LOCALISATION\n")
            next_time = 0
            for act in truck_route:

                current_time = next_time
                prev_event = "NONE"
                # Dealing with explicit truck events as given in dictionnary
                if act[0] != act[1]:
                    print(coords[act[0]], coords[act[1]])
                    f.write(
                        f"{current_time} ; DEPLACEMENT VEHICULE DESTINATION (LAT : {coords[act[1]][0]} ; LON : {coords[act[1]][1]}) ; (LAT : {coords[act[0]][0]} ; LON : {coords[act[0]][1]})\n"
                    )
                    prev_event = "DPLC"
                elif len(act) == 4 and (act[3] == "d1" or act[3] == "d2"):
                    f.write(
                        f"{current_time} ; RECHARGEMENT DRONE {act[3][-1]} ; (LAT : {coords[act[0]][0]} ; LON : {coords[act[0]][1]})\n"
                    )
                    prev_event = "RCHG" + act[3][-1]
                elif act[3] > 0:
                    for i in range(int(act[3])):
                        del_time = current_time + i*60
                        f.write(
                            f"{del_time} ; LIVRAISON COLIS ID : [à specifier] ; (LAT : {coords[act[0]][0]} ; LON : {coords[act[0]][1]})\n"
                        )
                next_time = current_time + act[2]

                # Dealing with implicit drone events happening in parallel to truck events
                for event in list(drone_events.keys()):
                    if drone_events[event] >= next_time:
                        break
                    d = int(event[0])
                    event_type = event[2:]
                    if event_type == "go":
                        f.write(
                            f"{drone_events[event]} ; LIVRAISON DRONE {d} COLIS ID : [à specifier]\n"
                        )
                    elif event_type == "back":
                        end_node = drone_routes[d][d_ix[d]][1]
                        f.write(
                            f"{drone_events[event]} ; RECUPERATION DRONE {d} ; (LAT : {coords[end_node][0]} ; LON : {coords[end_node][1]})\n"
                        )
                    else:
                        print(f"ERROR, {event_type} not valid!!!")
                    d_ix[d] += 1
                    drone_times.pop(event)


                # Dealing with implicit vehicle events based on previous event
                if prev_event == "DPLC":
                    f.write(
                        f"{next_time} ; ARRIVEE VEHICULE ; (LAT : {coords[act[1]][0]} ; LON : {coords[act[1]][1]})\n"
                    )
                elif prev_event == "RCHG1" or prev_event == "RCHG2":
                    str_d = prev_event[-1]
                    d = int(str_d)
                    f.write(
                        f"{next_time} ; LARGAGE DRONE {d} POUR LIVRAISON COLIS ID : [à spécifier] ; (LAT : {coords[act[1]][0]} ; LON : {coords[act[1]][1]})\n"
                    )
                    drone_events[str_d+"_go"] = next_time + drone_routes[d][d_ix[d]][2]
                    drone_events[str_d+"_back"] = next_time + drone_routes[d][d_ix[d]][2] + drone_routes[d][d_ix[d]+1][2]
                    drone_events = dict(sorted(drone_events.items(), key=lambda item: item[1]))

                # Dealing with implicit drone events happening at the exact current time
                for event in list(drone_events.keys()):
                    if drone_events[event] > next_time:
                        break
                    d = int(event[0])
                    event_type = event[2:]
                    if event_type == "go":
                        f.write(
                            f"{drone_events[event]} ; LIVRAISON DRONE {d} COLIS ID : [à specifier]\n"
                        )
                    # careful, recuperation can only be done at stop for case 2, correct that
                    elif event_type == "back":
                        end_node = drone_routes[d][d_ix[d]][1]
                        f.write(
                            f"{drone_events[event]} ; RECUPERATION DRONE {d} ; (LAT : {coords[end_node][0]} ; LON : {coords[end_node][1]})\n"
                        )
                    else:
                        print(f"ERROR, {event_type} not valid!!!")
                    d_ix[d] += 1
                    drone_events.pop(event)

        print(f"Solution written in {_sol_file}")
