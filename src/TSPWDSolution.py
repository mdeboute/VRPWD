from TSPWDData import TSPWDData
from pathlib import Path
import networkx as nx
import time
import matplotlib.pyplot as plt


class TSPWDSolution:
    __BASE_DIR = Path(__file__).resolve().parent.parent

    def __init__(
        self,
        instance: TSPWDData,
        algorithm: str,
        objective_value: int,
        solution,  # TODO: Define the type of decision_variables
    ):
        self.instance = instance
        self.algorithm = algorithm
        self.objective_value = objective_value
        self.solution = solution
        self.graph = self.graph_sol()
        self.plot_graph()
        self.__SOLUTION_DIR = str(self.__BASE_DIR) + "/solution/" + self.algorithm

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
    
    def graph_sol(self):
        """create a graph version of the solution with the nodes from the initial graph"""
        print("===========GRAPH SOL CREATION==============")
        start_time=time.time()
        #check for the type of vehicle which use the arc in order to use one color for truck and one color for drone
        print('algo = ',self.algorithm)
        print("solution = ",self.solution)
        if self.algorithm=='Nearest Neighbour Greedy' or self.algorithm=='MIP':
            vehicle='truck'
            truck_solution=self.solution
        else:
            #modified incoming solution to adjusted edges attributes for drone
            vehicle=None
            truck_solution=None
            drone_solution=None
        #create graph
        graph=nx.DiGraph()
        #add demand nodes
        for node in self.instance.demands_nodes :
            val_demand=self.instance.graph.nodes[node]['demand']
            coord=self.instance.graph.nodes[node]['coordinates']
            graph.add_node(node,coordinates=coord,depot=False, demand=val_demand)
        #add depot node
        graph_coord_depot=self.instance.graph.nodes[self.instance.depot]['coordinates']
        graph.add_node(self.instance.depot,coordinates=graph_coord_depot,depot=True,demand=0)
        #compute pcc between consecutive two nodes of the solution
        print("depot = ",self.instance.depot)
        for i, x in enumerate(truck_solution[:-1]):
            y = truck_solution[i+1]
            #print('demand node = ',x, y)
            pcc=nx.shortest_path(self.instance.graph,x,y,weight="travel_time",method="dijkstra")
            #print('pcc = ',pcc)
            #loop 2 to 2 over pcc
            for i2, x2 in enumerate(pcc[:-1]):
                y2 = pcc[i2+1]                
                #print('intermediate nodes = ',x2,y2)
                #check if y2 not already in nodes
                if not graph.has_node(y2):
                    #get y2 instance.graph coordinates
                    y2_coord=self.instance.graph.nodes[y2]['coordinates']
                    graph.add_node(y2,coordinates=y2_coord,depot=False,demand=0)
                graph.add_edge(x2,y2,vehicle=vehicle)
        end_time=time.time()
        processing_time=end_time-start_time
        print(graph)
        print("processing time = ",processing_time)
        return graph
    
    def plot_graph(self):
        """plot the graph"""
        print("==================== PLOT GRAPH ====================")
        # Draw graph
        coordinates = nx.get_node_attributes(self.graph, "coordinates")
        node_colors = []
        count = 0
        for node in self.graph.nodes():
            if self.graph.nodes[node]['depot']:
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
        # TODO: Check if the solution is feasible
        pass

    def write(self):
        # TODO:Write the solution to a file
        pass
