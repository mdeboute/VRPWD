from VRPWDData import VRPWDData
from TSPMIPModel import TSPMIPModel
from utils import verbose_print
from time import time
import networkx as nx

class VRPWDHeuristic_2:
    def __init__(self, instance: VRPWDData, number_of_drones: int):
        self.instance = instance
        self.ordened_demands_nodes = []
        self.number_of_drones=number_of_drones
        self.init_sol = TSPMIPModel(self.instance).solve()

    def first_stage(self):
        """ first stage part of the heuristic
        -given the TSP solution, affect the k drones to the k first nodes
        while the truck goes to the k+1th node etc"""
        print('deposit : ',self.instance.deposit)
        print('initial sol : ',self.init_sol)
        number_of_drones=2
        number_of_vehicles=number_of_drones+1
        dpd_nodes_per_vehicle={} #dict {0:[...], 1:[...], ..., k:[...]} with 0 is the truck dpd nodes list and the other drone dpd nodes list 
        print('solution : ',self.init_sol.solution['truck'])
        print('dpd nodes : ',self.instance.dpd_nodes)
        for i in range(len(self.init_sol.solution["truck"]) - 1):
            if self.init_sol.solution["truck"][i][1] in self.instance.dpd_nodes[1:] and self.init_sol.solution["truck"][i][1] not in self.ordened_demands_nodes:
                self.ordened_demands_nodes.append(self.init_sol.solution["truck"][i][1])
        self.ordened_demands_nodes.append(self.instance.deposit)
        self.ordened_demands_nodes.insert(0,self.instance.deposit)
        print('ordened demand node : ',self.ordened_demands_nodes)
        for i in range(number_of_vehicles):
            dpd_nodes_per_vehicle[i]=[]
            for node in self.ordened_demands_nodes:
                if self.ordened_demands_nodes.index(node)%number_of_vehicles==i:
                    dpd_nodes_per_vehicle[i].append(node)
        print(dpd_nodes_per_vehicle)
        
        return dpd_nodes_per_vehicle

    def second_stage(self,dpd_nodes_per_vehicle):
        """find the best node to launch drones within each sp between two truck visited demande nodes"""
        print("===============SECOND STAGE==============")
        #create dict to update the demandd
        solution={}
        demand_dict={}
        for node in self.instance.graph.nodes:
            if self.instance.graph.nodes[node]['demand'] >= 1:
                demand_dict[node]=self.instance.graph.nodes[node]['demand']
        print('demand_dict : ',demand_dict)
        #get each move that the truc will do in term of demand node
        pair_by_pair_truck_dpd_nodes = [(dpd_nodes_per_vehicle[0][i], dpd_nodes_per_vehicle[0][i + 1]) for i in range(len(dpd_nodes_per_vehicle[0])-1)]
        print('result = ',pair_by_pair_truck_dpd_nodes)
        #where to launch drones
        #loop over the sp
        #compute shortest path
        counter=0
        for src_dest in pair_by_pair_truck_dpd_nodes:
            print('index du node {} : {}'.format(src_dest[0],self.instance.dpd_nodes.index(src_dest[0])))
            print('index du node {} : {}'.format(src_dest[1],self.instance.dpd_nodes.index(src_dest[1])))
            self.instance.dpd_time_matrix[self.instance.dpd_nodes.index(src_dest[0])][self.instance.dpd_nodes.index(src_dest[0])]
            sp_truck=nx.shortest_path(self.instance.graph, src_dest[0],src_dest[1],weight='travel_time')
            sp_truck_tt=nx.shortest_path_length(self.instance.graph, src_dest[0],src_dest[1],weight='travel_time')
            print('pcc entre les deux : ',sp_truck)
            print('with a tt : {} s'.format(sp_truck_tt))
            #de quel noeud faire partir les drones?
            min_d1_tt=1000000000
            min_d2_tt=1000000000
            print('dpd_nodes_per_vehicle[1][counter] : ',dpd_nodes_per_vehicle[1][counter])
            print('dpd_nodes_per_vehicle[2][counter] : ',dpd_nodes_per_vehicle[2][counter])
            for current_truck_node in sp_truck:
                #trouver le noeud de lancement qui minimise le temps de parcourt pour chaque drone
                print('current_truck_node : ',current_truck_node)

                sp_d1=self.instance.drone_time_matrix[current_truck_node-1][dpd_nodes_per_vehicle[1][counter]-1]
                sp_d2=self.instance.drone_time_matrix[current_truck_node-1][dpd_nodes_per_vehicle[2][counter]-1]
                print('sp_d1 = ',sp_d1)
                print('sp_d2 = ',sp_d2)
                if sp_d1<min_d1_tt:
                    min_d1_tt=sp_d1
                    min_d1_node=current_truck_node
                if sp_d2<min_d2_tt:
                    min_d2_tt=sp_d2
                    min_d2_node=current_truck_node
            #here the best nodes are founded
            print('best d1 node : {} with {} s'.format(min_d1_node,min_d1_tt))
            print('best d2 node : {} with {} s'.format(min_d2_node,min_d2_tt))



            break
    def create_solution(self,solution,min_d1_node,min_d1_tt,min_d2_node,min_d2_tt,current_truck_node):
        """create the solution for one truck move """
        preparing_drone_time=30
        
        if min_d1_node==current_truck_node:
            #add 30s to prepare the drone
            real_time_1=nx.shortest_path_length(self.instance.graph,current_truck_node,min_d1_node,weight='travel_time')+preparing_drone_time
        if min_d2_node==current_truck_node:
            real_time_2=nx.shortest_path_length(self.instance.graph,current_truck_node,min_d1_node,weight='travel_time')+preparing_drone_time

    def solve(self):
        dpd_nodes_per_vehicle=self.first_stage()
        self.second_stage(dpd_nodes_per_vehicle)
