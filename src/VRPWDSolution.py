import networkx as nx
import matplotlib.pyplot as plt
import time

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
            graph.add_node(node, coordinates=coords, deposit=False, demand=demand_value)
        # add deposit node
        graph_coords_deposit = self.instance.graph.nodes[self.instance.deposit][
            "coordinates"
        ]
        graph.add_node(
            self.instance.deposit,
            coordinates=graph_coords_deposit,
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
                    graph.add_node(
                        dest, coordinates=dest_coords, deposit=False, demand=0
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
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)
        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"

        truck_shift = list(self.solution["truck"])
        drone_1_shift = list(self.solution["drone_1"])
        drone_2_shift = list(self.solution["drone_2"])
        coords = nx.get_node_attributes(self.graph, "coordinates")

        # with open(_sol_file, "w") as f:
        #     f.write("TEMPS ; EVENEMENT ; LOCALISATION")

        pass
