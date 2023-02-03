import networkx as nx
import matplotlib.pyplot as plt
import time
import osmnx as ox
import folium
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
        solution: dict,
        verbose: bool,
    ):
        self.__VERBOSE = verbose
        global vprint
        vprint = verbose_print(self.__VERBOSE)

        self.instance = instance
        self.algorithm = algorithm
        self.objective_value = objective_value
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
        drone_1_tour = self.solution["drone_1"]
        drone_2_tour = self.solution["drone_2"]
        # create graph
        graph = nx.DiGraph()
        # add demands nodes
        for node in self.instance.dpd_nodes[1:]:
            demand_value = self.instance.graph.nodes[node]["demand"]
            coords = self.instance.graph.nodes[node]["coordinates"]
            inversed_coords = (coords[1], coords[0])
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
        # case 0 -> only truck tour
        if self.instance._CASE == 0:
            for move in truck_tour:
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
                graph.add_edge(src, dest, travel_time=tt)
        # case 1,2,3 -> truck and drones
        else:
            print("ERROR: case 1,2,3 not implemented yet!")
            pass
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("graph: ", graph)
        vprint("processing time: ", processing_time)
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

    def save_html(self):
        """Save the solution on a html map"""

        vprint("=============== SAVE HTML ===============")
        # Coordonnées de la position centrale
        start_time = time.time()
        x, y = (44.8464, 0.5703)

        # Créer le graphe à partir de la position centrale et une distance de 5 km
        init_graph = ox.graph_from_point(
            center_point=(x, y), dist=5000, network_type="drive"
        )

        # creer le sous graph avec les noeuds qui nous intéressent
        list_nodes = []
        for node in self.graph.nodes:
            # trouver le noeud correspondant dans le init graph omsnx
            coords_sol = (
                round(self.graph.nodes[node]["coordinates"][1], 7),
                round(self.graph.nodes[node]["coordinates"][0], 7),
            )
            nn = ox.nearest_nodes(init_graph, coords_sol[1], coords_sol[0]) #lon,lat
            list_nodes.append(nn)
        # create the subgraph
        subgraph = init_graph.subgraph(list_nodes)

        # Afficher le sous graphe
        m = ox.folium.plot_graph_folium(subgraph)
        # save the map
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)
        m.save(self.__SOLUTION_DIR + "/solution.html")
        vprint("HTML Map saved in {} ".format(self.__SOLUTION_DIR + "/solution.html"))
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing time: ", processing_time)

    def check(self):
        if self.instance._CASE == 0:
            # check if all demand nodes are in the tour
            for node in self.instance.dpd_nodes[1:]:
                if node not in [tuple[0] for tuple in list(self.solution["truck"])]:
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
            if [tuple[0] for tuple in list(self.solution["truck"])].count(
                self.instance.deposit
            ) > 2:
                print("ERROR: tour visits the deposit twice!")
                return False
        else:
            print("ERROR: checks for that case are not implemented yet!")
        return True

    def write(self):
        # with our solution format, we assume that the delivery guy left the demand node just after arriving
        # and that he arrived at the next node at time: delivery_time + travel_time
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)
        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"

        truck_shift = list(self.solution["truck"])
        drone_1_shift = list(self.solution["drone_1"])
        drone_2_shift = list(self.solution["drone_2"])
        coords = nx.get_node_attributes(self.graph, "coordinates")

        with open(_sol_file, "w") as f:
            f.write("TEMPS ; EVENEMENT ; LOCALISATION\n")
            current_time = 0
            if self.instance._CASE == 0:
                for move in truck_shift:
                    f.write(
                        f"{current_time} ; DEPLACEMENT VEHICULE DESTINATION (LAT : {coords[move[0]][0]} ; LON : {coords[move[0]][1]}) ; (LAT : {coords[move[1]][0]} ; LON : {coords[move[1]][1]})\n"
                    )
                    current_time = move[2]
                    f.write(
                        f"{current_time} ; ARRIVEE VEHICULE ; (LAT : {coords[move[1]][0]} ; LON : {coords[move[1]][1]})\n"
                    )
                    if move[1] in self.instance.dpd_nodes[1:]:
                        f.write(
                            f"{current_time} ; LIVRAISON COLIS ID : {move[1]} ; (LAT : {coords[move[1]][0]} ; LON : {coords[move[1]][1]})\n"
                        )
                        current_time = move[2] + 60
        print(f"Solution written in {_sol_file}")
