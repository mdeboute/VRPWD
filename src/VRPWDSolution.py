from VRPWDData import VRPWDData
from pathlib import Path
import networkx as nx
import matplotlib.pyplot as plt
from utils import verbose_print
import time


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
        """create the graph format of the solution"""
        vprint("===================CREATE GRAPH SOLUTION====================")
        start_time=time.time()
        #get solutions for each type  of vehicle
        truck_tour=self.solution['truck']
        drone_1_tour=self.solution['drone_1']
        drone_2_tour=self.solution['drone_2']
        #create graph
        graph=nx.DiGraph()
        #add demande nodes
        for node in self.instance.dpd_nodes:
            demand_value=self.instance.graph.nodes[node]['demand']
            coord=self.instance.graph.nodes[node]['coordinates']
            graph.add_node(node,coordinates=coord,deposit=False, demand=demand_value)
        #add deposit node
        graph_coord_deposit=self.instance.graph.nodes[self.instance.deposit]['coordinates']
        graph.add_node(self.instance.deposit,coordinates=graph_coord_deposit,deposit=True,demand=0)
        #case 0 -> only truck tour
        if self.instance._CASE < 1:
            vprint(truck_tour)
            for move in truck_tour:
                src=move[0]
                dest=move[1]
                tt=move[2]
                #check if the node already exists
                if not graph.has_node(dest):
                    #by construction, this node is not the deposit nor a demand node
                    #create the node
                    dest_coords=self.instance.graph.nodes[dest]['coordinates']
                    graph.add_node(dest,coordinates=dest_coords,deposit=False,demand=0)
                graph.add_edge(src,dest,travel_time=tt)
        #case 1,2,3 -> truck and drones
        else:
            pass
        end_time=time.time()
        processing_time=end_time-start_time
        vprint('graph : ',graph)
        vprint('processing time : ',processing_time)
        return graph

    def plot(self):
        """plot the graph"""
        print("==================== PLOT GRAPH ====================")
        # Draw graph
        coordinates = nx.get_node_attributes(self.graph, "coordinates")
        node_colors = []
        count = 0
        for node in self.graph.nodes():
            if self.graph.nodes[node]['deposit']:
                node_colors.append("g")
            elif self.graph.nodes[node]["demand"] > 0:
                node_colors.append("r")
                count += 1
            else:
                node_colors.append("b")
        nx.draw(self.graph, coordinates, node_color=node_colors, with_labels=True)
        # Show plot
        print("number_of_demand_nodes = ", count)
        plt.show()

    def check(self):
        pass

    def write(self):
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)

        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"

        with open(_sol_file, "w") as f:
            pass
        pass


# TODO: implement this class
