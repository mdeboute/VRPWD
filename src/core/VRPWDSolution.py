import networkx as nx
import matplotlib.pyplot as plt
import folium

from pathlib import Path
from core.utils import verbose_print
from core.VRPWDData import VRPWDData
from pathlib import Path
from itertools import chain
from shapely.geometry import Point, LineString


class VRPWDSolution:
    __BASE_DIR = Path(__file__).resolve().parent.parent.parent

    def __init__(
        self,
        instance: VRPWDData,
        algorithm: str,
        objective_value: int,
        runtime: float,
        gap,
        solution: dict,
        verbose: bool,
    ):
        self._VERBOSE = verbose
        global vprint
        vprint = verbose_print(self._VERBOSE)

        self.instance = instance
        self.algorithm = algorithm
        self.objective_value = objective_value
        self.runtime = runtime
        self.gap = gap
        self.solution = solution
        self.graph = self._create_graph()

        self.__SOLUTION_DIR = (
            str(self.__BASE_DIR)
            + "/solution/"
            + self.instance._INSTANCE_NAME
            + "/case_"
            + str(self.instance._CASE)
            + "/"
        )

    def __str__(self):
        return f"Solution(objective_value={self.objective_value})"

    def __repr__(self):
        return self.__str__()

    def _create_graph(self):
        """Create the graph format of the solution"""

        vprint("=================== CREATE GRAPH SOLUTION ===================")
        # get solutions for each type  of vehicle
        truck_tour = self.solution["truck"]
        number_of_drones = 2
        # create graph
        graph = nx.DiGraph()
        # add demands nodes
        for node in self.instance.dpd_nodes[1:]:
            demand_value = self.instance.graph.nodes[node]["demand"]
            coords = self.instance.graph.nodes[node]["coordinates"]
            inversed_coords = (coords[1], coords[0])  # another networkx curiosity 0_0??
            graph.add_node(
                node,
                coordinates=inversed_coords,
                deposit=False,
                intermediate=False,
                demand=demand_value,
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
            intermediate=False,
            demand=0,
        )
        # any case -> truck tour
        for move in truck_tour:
            # check for a mouvement tuple
            if len(move) == 3 and move[0] != move[1]:
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
                        dest,
                        coordinates=inversed_dest_coords,
                        deposit=False,
                        intermediate=False,
                        demand=0,
                    )
                graph.add_edge(src, dest, travel_time=tt, vehicle="truck")
        # case 1,2,3 -> add drone edges
        if self.instance._CASE > 0:
            for i in range(number_of_drones):
                drone_tour = self.solution["drone_{}".format(i + 1)]
                for move in drone_tour:
                    # check for a drone move
                    if len(move) == 3 and move[0] != move[1]:
                        src = move[0]
                        dest = move[1]
                        tt = move[2]
                        graph.add_edge(
                            src, dest, travel_time=tt, vehicle="drone_{}".format(i + 1)
                        )
        vprint("graph:", graph)
        return graph

    def plot(self, graph=None):
        """Plot the solution graph"""

        vprint("==================== PLOT GRAPH ====================")
        if graph == None:
            graph = self.graph
        # Draw graph
        coordinates = nx.get_node_attributes(graph, "coordinates")
        node_colors = []
        edge_colors = []
        for node in graph.nodes():
            if graph.nodes[node]["deposit"]:
                node_colors.append("g")
            elif graph.nodes[node]["demand"] > 0:
                node_colors.append("r")
            else:
                node_colors.append("b")
        for _, _, d in graph.edges(data=True):
            # print(d)
            if d["vehicle"] == "truck":
                edge_colors.append("black")
            elif d["vehicle"] == "drone_1":
                edge_colors.append("green")
            elif d["vehicle"] == "drone_2":
                edge_colors.append("yellow")
        nx.draw(
            graph,
            coordinates,
            node_color=node_colors,
            edge_color=edge_colors,
            with_labels=False,
            node_size=50,
            width=0.5,
        )
        # Show plot
        plt.show()

    def _create_artificial_graph(self, L):
        """Create the artificial graph (i.e. segmented edges) from the solution graph"""

        graph_copy = self.graph.copy()
        graph_copy_2 = self.graph.copy()

        # Cycle through all arcs in the graph
        for u, v, data in graph_copy.out_edges(data=True):
            # Calculate arc length
            is_truck = False
            length = data["travel_time"]
            if data["vehicle"] == "truck":
                is_truck = True

            # If the arc is longer than the threshold length L
            if is_truck and length > L:
                # Calculate the number of sub-arcs needed
                num_segments = int(length / L)
                # Create the dividing points
                points = [graph_copy.nodes[u]["coordinates"]]
                for i in range(1, num_segments):
                    frac = i / num_segments
                    point = LineString(
                        [
                            graph_copy.nodes[u]["coordinates"],
                            graph_copy.nodes[v]["coordinates"],
                        ]
                    ).interpolate(frac, normalized=True)
                    points.append((point.x, point.y))
                points.append(graph_copy.nodes[v]["coordinates"])
                # Add new nodes for each dividing point
                for i in range(len(points) - 1):
                    coordinates = (
                        points[i][0],
                        points[i][1],
                    )  # Warning inversion of latitude and longitude
                    new_nodes = []
                    if i == 0:
                        new_u = u
                    else:
                        new_u = f"split_{u}_{i}"
                        new_nodes.append((new_u, coordinates))
                    if i == len(points) - 2:
                        new_v = v
                    else:
                        new_v = f"split_{u}_{i+1}"
                        new_nodes.append((new_v, (points[i + 1][0], points[i + 1][1])))
                    # Add arcs between new nodes
                    for node in new_nodes:
                        graph_copy_2.add_node(
                            node[0],
                            coordinates=node[1],
                            deposit=False,
                            demand=0,
                            intermediate=True,
                        )
                    graph_copy_2.add_edge(new_u, new_v, **data)
                # Delete original arc
                graph_copy_2.remove_edge(u, v)
        return graph_copy_2

    def save_sol_html(self):
        """Plot the solution on a dynamic html map"""

        graph = self._create_artificial_graph(4)
        intermediate_nodes = []
        for node, data in graph.nodes(data=True):
            if data["intermediate"]:
                intermediate_nodes.append(node)
        # convert graphics data from NetworkX to Shapely geometry objects
        positions = {
            k: (v[1], v[0])
            for k, v in nx.get_node_attributes(graph, "coordinates").items()
        }
        points = {node: Point(positions[node]) for node in positions}
        lines_d1 = [
            LineString([positions[u], positions[v]])
            for u, v, d in graph.edges(data=True)
            if d["vehicle"] == "drone_1"
        ]
        lines_d2 = [
            LineString([positions[u], positions[v]])
            for u, v, d in graph.edges(data=True)
            if d["vehicle"] == "drone_2"
        ]
        lines_truck = [
            LineString([positions[u], positions[v]])
            for u, v, d in graph.edges(data=True)
            if d["vehicle"] == "truck"
        ]
        # Create a map with Folium
        m = folium.Map(location=list(positions.values())[0], zoom_start=10)

        # Add lines to represent roads
        for line in lines_d1:
            coords = list(line.coords)
            folium.PolyLine(coords, color="green", weight=2, opacity=1).add_to(m)
        for line in lines_d2:
            coords = list(line.coords)
            folium.PolyLine(coords, color="yellow", weight=2, opacity=1).add_to(m)
        for line in lines_truck:
            coords = list(line.coords)
            folium.PolyLine(coords, color="black", weight=2, opacity=1).add_to(m)

        # Ajouter des marqueurs pour représenter les nœuds de demande
        for node, point in points.items():
            if node == self.instance.deposit:
                color = "green"
                folium.Marker(
                    list(point.coords)[0],
                    icon=folium.Icon(color=color),
                    tooltip=f"{node}",
                ).add_to(m)
            elif node in self.instance.dpd_nodes:
                color = "red"
                folium.Marker(
                    list(point.coords)[0],
                    icon=folium.Icon(color=color),
                    tooltip=f"{node}",
                ).add_to(m)
            elif node in intermediate_nodes:
                pass
            else:
                color = "blue"
                folium.Marker(
                    list(point.coords)[0],
                    icon=folium.Icon(color=color),
                    tooltip=f"{node}",
                ).add_to(m)
        # save the map
        _assets_path = "assets/" + self.instance._INSTANCE_NAME
        Path(_assets_path).mkdir(parents=True, exist_ok=True)
        m.save(_assets_path + "/solution.html")
        print(f"HTML Map saved in {_assets_path}/solution.html")

    def _get_vistited_nodes(self):
        nodes_visited_by_truck = []
        nodes_visited_by_drone_1 = []
        nodes_visited_by_drone_2 = []
        for move in self.solution["truck"]:
            if len(move) == 3 and move[0] != move[1]:
                nodes_visited_by_truck.append(move[0])
        for move in self.solution["drone_1"]:
            nodes_visited_by_drone_1.append(move[0])
        for move in self.solution["drone_2"]:
            nodes_visited_by_drone_2.append(move[0])
        visited_nodes = [
            nodes_visited_by_truck,
            nodes_visited_by_drone_1,
            nodes_visited_by_drone_2,
        ]
        return visited_nodes

    def check(self):
        visited_nodes = self._get_vistited_nodes()

        def _classic_check(self):
            # check if all demand nodes are in the tour
            for node in self.instance.dpd_nodes[1:]:
                if node not in list(set(list(chain(*visited_nodes)))):
                    print(f"ERROR: demand node {node} is not in the tour!")
                    print(self.solution)
                    return False
            # check that we start at the deposit
            if self.solution["truck"][0][0] != self.instance.deposit:
                print("ERROR: tour does not start at the deposit!")
                print(self.solution)
                return False
            # check that we end at the deposit
            if self.solution["truck"][-1][1] != self.instance.deposit:
                print("ERROR: tour does not end at the deposit!")
                print(self.solution)
                return False
            # check that we do not visit the deposit twice
            # count the number of times we visit the deposit in the nodes visited by the truck
            if visited_nodes[0].count(self.instance.deposit) > 1:
                print("WARNING: we visit the deposit more than once!")
                print(self.solution)
                return True
            # check that the 2nd element of the tuple always equals the 1st element of the next tuple
            for i in range(len(self.solution["truck"]) - 1):
                if self.solution["truck"][i][1] != self.solution["truck"][i + 1][0]:
                    print("ERROR: the tour is not continuous!")
                    # print the 2 tuples that are not continuous
                    print(self.solution["truck"][i])
                    print(self.solution["truck"][i + 1])
                    return False

        if self.instance._CASE == 0:
            _classic_check(self)
        elif self.instance._CASE == 1:
            _classic_check(self)
        elif self.instance._CASE == 2:
            _classic_check(self)
        elif self.instance._CASE == 3:
            _classic_check(self)
        else:
            print("ERROR: unknown case")
            print("Please use -h or --help to see the usage")
            return False
        return True

    def write(self):
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)
        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"

        truck_route = list(self.solution["truck"])
        drone_routes = {
            1: list(self.solution["drone_1"]),
            2: list(self.solution["drone_2"]),
        }
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
                    f.write(
                        f"{current_time} ; DEPLACEMENT VEHICULE DESTINATION (LAT : {coords[act[1]][0]} ; LON : {coords[act[1]][1]}) ; (LAT : {coords[act[0]][0]} ; LON : {coords[act[0]][1]})\n"
                    )
                    prev_event = "DPLC"
                elif len(act) == 4 and (act[3] == "d1" or act[3] == "d2"):
                    f.write(
                        f"{current_time} ; CHARGEMENT DRONE {act[3][-1]} ; (LAT : {coords[act[0]][0]} ; LON : {coords[act[0]][1]})\n"
                    )
                    prev_event = "RCHG" + act[3][-1]
                elif len(act) == 4 and act[3] > 0:
                    for i in range(int(act[3])):
                        f.write(
                            f"{current_time} ; LIVRAISON COLIS ID : {act[1]} ; (LAT : {coords[act[0]][0]} ; LON : {coords[act[0]][1]})\n"
                        )
                next_time = current_time + act[2]

                # Dealing with implicit drone events happening in parallel to truck events
                for event in list(drone_events.keys()):
                    if drone_events[event] > next_time:
                        break  # If all remaining events happen after current period, we don't writing them
                    d = int(event[0])
                    event_type = event[2:]
                    if event_type == "go":
                        f.write(
                            f"{drone_events[event]} ; LIVRAISON DRONE {d} COLIS ID : {drone_routes[d][d_ix[d]][1]}\n"
                        )
                    elif event_type == "back" and (
                        self.instance._CASE == 3
                        or act[0] == drone_routes[d][d_ix[d]][1] == act[1]
                    ):
                        # A drone can only be recovered midway through a truck action if the truck is stationned on a node or we are in case 3
                        end_node = drone_routes[d][d_ix[d]][1]
                        f.write(
                            f"{drone_events[event]} ; RECUPERATION DRONE {d} ; (LAT : {coords[end_node][0]} ; LON : {coords[end_node][1]})\n"
                        )
                    else:
                        continue
                    d_ix[d] += 1
                    drone_events.pop(event)

                # Dealing with implicit vehicle events based on previous event
                if prev_event == "DPLC":
                    f.write(
                        f"{next_time} ; ARRIVEE VEHICULE ; (LAT : {coords[act[1]][0]} ; LON : {coords[act[1]][1]})\n"
                    )
                elif prev_event == "RCHG1" or prev_event == "RCHG2":
                    str_d = prev_event[-1]
                    d = int(str_d)
                    f.write(
                        f"{next_time} ; LARGAGE DRONE {d} POUR LIVRAISON COLIS ID : {drone_routes[d][d_ix[d]][1]} ; (LAT : {coords[act[1]][0]} ; LON : {coords[act[1]][1]})\n"
                    )
                    drone_events[str_d + "_go"] = (
                        next_time + drone_routes[d][d_ix[d]][2]
                    )
                    drone_events[str_d + "_back"] = (
                        next_time
                        + drone_routes[d][d_ix[d]][2]
                        + drone_routes[d][d_ix[d] + 1][2]
                    )
                    drone_events = dict(
                        sorted(drone_events.items(), key=lambda item: item[1])
                    )

                # Dealing with implicit drone events happening at the exact current time
                # (only those that were inelligible during previous for loop over drone_events)
                for event in list(drone_events.keys()):
                    if drone_events[event] > next_time:
                        break
                    d = int(event[0])
                    event_type = event[2:]
                    if event_type == "go":
                        f.write(
                            f"{next_time} ; LIVRAISON DRONE {d} COLIS ID : {drone_routes[d][d_ix[d]][1]}\n"
                        )
                    elif event_type == "back" and drone_routes[d][d_ix[d]][1] == act[1]:
                        end_node = drone_routes[d][d_ix[d]][1]
                        f.write(
                            f"{next_time} ; RECUPERATION DRONE {d} ; (LAT : {coords[end_node][0]} ; LON : {coords[end_node][1]})\n"
                        )
                    else:
                        continue
                    d_ix[d] += 1
                    drone_events.pop(event)

        print(f"Solution written in {_sol_file}")
