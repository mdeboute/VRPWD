from VRPWDData import VRPWDData
from pathlib import Path
import networkx as nx
import time
import matplotlib.pyplot as plt
from utils import v_print


class VRPWDSolution:
    __BASE_DIR = Path(__file__).resolve().parent.parent

    def __init__(
        self,
        instance: VRPWDData,
        algorithm: str,
        objective_value: int,
        solution,
        verbose: bool,
    ):
        self.__VERBOSE = verbose
        global vprint
        vprint = v_print(self.__VERBOSE)

        self.instance = instance
        self.algorithm = algorithm
        self.objective_value = objective_value
        self.solution = solution
        self.graph = self._create_graph()
        self.tour = self._create_tour()

        self.__SOLUTION_DIR = (
            str(self.__BASE_DIR) + "/solution/" + self.instance._INSTANCE_NAME + "/"
        )

    def __str__(self):
        return f"Solution(objective_value={self.objective_value})"

    def __repr__(self):
        return self.__str__()

    def print_tour(self):
        # print the tour
        print(f"Tour (cost={self.objective_value}): ", end="")
        for i in self.solution:
            print(i, end=" ")
        print()

    def _create_graph(self):
        """Create the final solution graph version"""

        vprint("=========== GRAPH FINAL SOLUTION CREATION ===========")
        # check for the type of vehicle which use the arc in order to use one color for truck and one color for drone
        vprint("algo = ", self.algorithm)
        vprint("solution = ", self.solution)
        tour = []
        if self.instance._CASE < 1:
            _vehicle = "truck"
            _truck_solution = self.solution
        else:
            # modified incoming solution to adjusted edges attributes for drone
            _vehicle = None
            _truck_solution = None
            _drone_solution = None
        # create graph
        graph = nx.DiGraph()
        # add demand nodes
        for node in self.instance.dpd_nodes:
            val_demand = self.instance.graph.nodes[node]["demand"]
            coord = self.instance.graph.nodes[node]["coordinates"]
            graph.add_node(node, coordinates=coord, depot=False, demand=val_demand)
        # add depot node
        graph_coord_depot = self.instance.graph.nodes[self.instance.deposit][
            "coordinates"
        ]
        graph.add_node(
            self.instance.deposit, coordinates=graph_coord_depot, depot=True, demand=0
        )
        # compute sp between consecutive two nodes of the solution
        for i, x in enumerate(_truck_solution[:-1]):
            y = _truck_solution[i + 1]
            sp = nx.shortest_path(
                self.instance.graph, x, y, weight="travel_time", method="dijkstra"
            )
            # loop 2 to 2 over sp
            for i2, x2 in enumerate(sp[:-1]):
                y2 = sp[i2 + 1]
                # check if y2 not already in nodes
                if not graph.has_node(y2):
                    # get y2 instance.graph coordinates
                    y2_coord = self.instance.graph.nodes[y2]["coordinates"]
                    graph.add_node(y2, coordinates=y2_coord, depot=False, demand=0)
                graph.add_edge(x2, y2, vehicle="truck")
        return graph

    def _create_tour(self):
        """Create the final solution (= 1 tour) list version"""

        if self.instance._CASE < 1:
            _vehicle = "truck"
            _truck_solution = self.solution
        else:
            # modified incoming solution to adjusted edges attributes for drone
            _vehicle = None
            _truck_solution = None
            _drone_solution = None
        tour = []
        for i, x in enumerate(_truck_solution[:-1]):
            y = _truck_solution[i + 1]
            sp = nx.shortest_path(
                self.instance.graph, x, y, weight="travel_time", method="dijkstra"
            )
            # create final solution
            if len(tour) == 0:
                tour = sp
            else:
                sp.pop(0)
                tour = tour + sp
        return tour

    def plot(self):
        """Plot the graph"""

        vprint("==================== PLOT GRAPH ====================")
        # Draw graph
        coordinates = nx.get_node_attributes(self.graph, "coordinates")
        node_colors = []
        for node in self.graph.nodes():
            if self.graph.nodes[node]["depot"]:
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
        )
        # Show plot
        vprint("number_of_demand_nodes = ", len(self.instance.dpd_nodes) - 1)
        plt.show()

    def check(self):
        # TODO: Check if the solution is feasible
        pass

    def write(self):
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)

        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"
        _time_cpt = 0
        _coords = nx.get_node_attributes(self.graph, "coordinates")

        with open(_sol_file, "w") as f:
            f.write("TEMPS ; EVENEMENT ; LOCALISATION\n")
            for i in range(len(self.tour) - 1):
                f.write(
                    f"{_time_cpt} ; DEPLACEMENT VEHICULE DESTINATION (LAT : {_coords[self.tour[i]][0]} ; LON : {_coords[self.tour[i]][1]}) ; (LAT : {_coords[self.tour[i+1]][0]} ; LON : {_coords[self.tour[i+1]][1]})\n"
                )
                _time_cpt += self.instance.graph.edges[self.tour[i], self.tour[i + 1]][
                    "travel_time"
                ]

                f.write(
                    f"{_time_cpt} ; ARRIVEE VEHICULE ; (LAT : {_coords[self.tour[i+1]][0]} ; LON : {_coords[self.tour[i+1]][1]})\n"
                )
                if (
                    self.tour[i + 1] in self.instance.dpd_nodes[1:]
                    and self.instance._CASE < 1
                ):
                    f.write(
                        f"{_time_cpt} ; LIVRAISON COLIS ID : {self.tour[i+1]} ; (LAT : {_coords[self.tour[i+1]][0]} ; LON : {_coords[self.tour[i+1]][1]})\n"
                    )
        print(f"Solution written in {_sol_file}")
