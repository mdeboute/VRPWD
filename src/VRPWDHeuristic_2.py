from core.VRPWDData import VRPWDData
from algorithms.tsp.TSPMIPModel import TSPMIPModel
from core.utils import verbose_print
import time
import networkx as nx
from core.VRPWDSolution import VRPWDSolution


class VRPWDHeuristic_2:
    def __init__(self, instance: VRPWDData, number_of_drones: int, policy: str):
        self.instance = instance
        self._available_policies=['drone_inter', 'truck_inter','mix']
        self.policy=policy
        self._VERBOSE = self.instance._VERBOSE
        global vprint
        vprint = verbose_print(self._VERBOSE)
        if self.policy not in self._available_policies:
            raise Exception("ERROR, this policy is not available")
        self.__algorithm = "Greedy"
        self.ordoned_demands_nodes = []
        self.number_of_drones = number_of_drones
        self.init_sol = TSPMIPModel(self.instance).solve()

    def first_stage_v2(self,tour=None):
        """create the vehicule-node affectation dictionary from a tour if given or from OPT(TSP) otherwise"""
        #vprint("deposit : ", self.instance.deposit)
        super_nodes_dict={}
        demand_dict = {}
        for node in self.instance.graph.nodes:
            if self.instance.graph.nodes[node]["demand"] >= 1:
                demand_dict[node] = self.instance.graph.nodes[node]["demand"]
            if self.instance.graph.nodes[node]["demand"] >1:
                super_nodes_dict[node] =self.instance.graph.nodes[node]["demand"]
        self.super_nodes_dict=super_nodes_dict
        #vprint("dpd nodes : ", self.instance.dpd_nodes)
        #vprint('len(demand_node) : ',len(self.instance.dpd_nodes))
        #vprint('super node : ',super_nodes_dict)
        #vprint('len super node : ',len(super_nodes_dict))
        if tour==None:
            #vprint("initial sol : ", self.init_sol)
            #vprint("solution : ", self.init_sol.solution["truck"])
            for i in range(len(self.init_sol.solution["truck"]) - 1):
                if (
                    self.init_sol.solution["truck"][i][1] in self.instance.dpd_nodes[1:]
                    and self.init_sol.solution["truck"][i][1]
                    not in self.ordoned_demands_nodes
                ):
                    self.ordoned_demands_nodes.append(self.init_sol.solution["truck"][i][1])
            #in case of deposit is a demand node
            if self.instance.deposit not in self.ordoned_demands_nodes:
                self.ordoned_demands_nodes.insert(0, self.instance.deposit)
            self.ordoned_demands_nodes.append(self.instance.deposit)
        else:
            self.ordoned_demands_nodes=tour
        #vprint("ordened demand node : ", self.ordoned_demands_nodes)
        ordoned_super_nodes=[k for k in self.ordoned_demands_nodes if k in super_nodes_dict.keys()]
        self.ordoned_super_nodes=ordoned_super_nodes
        #vprint('ordoned super nodes : ',ordoned_super_nodes)
        node_vehicle_dict={}
        begin=0
        for node in self.ordoned_demands_nodes:
            if node in self.ordoned_super_nodes or node==self.instance.deposit:
                node_vehicle_dict[node]=0
                begin=1
            else:
                node_vehicle_dict[node]=begin
                begin+=1
                if begin==3:
                    begin=0
        #vprint('node vehicle dict : ',node_vehicle_dict)
        self.node_vehicle_dict=node_vehicle_dict
        return node_vehicle_dict

    def second_stage_v2(self,node_vehicle_dict):
        """create the solution"""
        # create dict to update the demandd
        solution = {"truck": []}
        for i in range(1, self.number_of_drones + 1):
            solution.setdefault("drone_{}".format(i), [])
        #vprint(solution)
        demand_dict = {}
        for node in self.instance.graph.nodes:
            if self.instance.graph.nodes[node]["demand"] >= 1:
                demand_dict[node] = self.instance.graph.nodes[node]["demand"]
        #vprint("demand_dict : ", demand_dict)
        truck_pair_dpd_nodes=[]
        for node in self.ordoned_demands_nodes:
            node_vehicle=node_vehicle_dict[node]
            if node_vehicle==0:
                truck_pair_dpd_nodes.append(node)
                #vprint('truck_pair_dpd_nodes : ',truck_pair_dpd_nodes)
                if len(truck_pair_dpd_nodes)==2:
                    sp_truck=nx.shortest_path(self.instance.graph,truck_pair_dpd_nodes[0],truck_pair_dpd_nodes[-1],weight="travel_time")
                    entire_dpd_nodes=[x for x in self.ordoned_demands_nodes if self.ordoned_demands_nodes.index(x)>=self.ordoned_demands_nodes.index(truck_pair_dpd_nodes[0]) 
                    and self.ordoned_demands_nodes.index(x)<=self.ordoned_demands_nodes.index(truck_pair_dpd_nodes[1])]
                    if node !=self.instance.deposit and entire_dpd_nodes[-1]==self.instance.deposit:
                        entire_dpd_nodes.pop(-1)
                    #we arrive at the end list-> entire dpd node is the starting truck node until the end of the list
                    if node==self.instance.deposit:
                        entire_dpd_nodes=self.ordoned_demands_nodes[self.ordoned_demands_nodes.index(truck_pair_dpd_nodes[0]):]
                    #vprint('ordoned demand nodes : ',self.ordoned_demands_nodes)
                    #vprint('ordoned super nodes : ',self.ordoned_super_nodes)
                    #vprint('entire_dpd_nodes : ',entire_dpd_nodes)
                    #vprint('------NEW TRUCK DEST------')
                    drone_target=[None for i in range(self.number_of_drones)]
                    go_tt_list=[None for i in range(self.number_of_drones)]
                    back_tt_list=[None for i in range(self.number_of_drones)]
                    #if there is drone to launch
                    if len(entire_dpd_nodes)>2:
                        for node in entire_dpd_nodes:
                            if node not in truck_pair_dpd_nodes:
                                node_vehicle=node_vehicle_dict[node]
                                drone_target[node_vehicle-1]=node
                                #compute travel time
                                go=self.instance.drone_time_matrix[truck_pair_dpd_nodes[0]-1][node-1]
                                go_tt_list[node_vehicle-1]=go
                                back=self.instance.drone_time_matrix[node-1][truck_pair_dpd_nodes[1]-1]
                                back_tt_list[node_vehicle-1]=back
                    #vprint('drone target : ',drone_target)
                    #vprint('go_tt_list : ',go_tt_list)
                    #vprint('back_tt_list : ',back_tt_list)
                    #vprint('all demand satisfied ? : ',all(v==0 for v in demand_dict.values()))
                    #vprint(demand_dict)
                    self.create_solution(solution,sp_truck,drone_target,go_tt_list,back_tt_list,demand_dict)
                    truck_pair_dpd_nodes=[truck_pair_dpd_nodes[-1]]
        #vprint('is all demand satisfied ? : ',all(v==0 for v in demand_dict.values()))
        is_feasible=all(v==0 for v in demand_dict.values())
        #if is_feasible:
            #print('-----------------------------------------')
            #print('node vehicle dict : ',node_vehicle_dict)
            #print('solution : ',solution)
        return (solution,is_feasible)

    def create_solution(
        self,
        solution,
        sp_truck,
        drone_target_list,
        go_tt_list,
        back_tt_list,
        demand_dict
    ):
        """create the solution for truck and drone, for current sp truck move"""
        #vprint("-------CREATE SOLUTION -------")
        preparing_drone_time = 30
        delivering_truck_time = 60
        launched_drones=[False for i in range(self.number_of_drones)]
        truck_chrono=[0 for i in range(1,self.number_of_drones+1)]
        truck_destination = sp_truck[-1]
        #vprint("demand_dict : ", demand_dict)
        #vprint("truck destination : ", truck_destination)
        #vprint('truck tt = ',truck_tt)
        #if there is drone to launch
        if not all(t==None for t in drone_target_list):
            #check if there is a drone demand node over the truck sp -> the reaction depends of the policy
            for node in sp_truck:
                if node in drone_target_list:
                    drone_number=drone_target_list.index(node)+1
                    #vprint('the node {} (for d{}) is in the truck sp'.format(node,drone_number))
                    if self.policy=='truck_inter':
                        pass
                    elif self.policy=='mix':
                        pass
            #drone which are not launched yet (ie drone inter policy and other)
            for d in launched_drones:
                if not d and drone_target_list[launched_drones.index(d)]!=None:
                    drone_number=launched_drones.index(d)+1
                    #create truck launch event
                    lauching_drone_event=(sp_truck[0],sp_truck[0],preparing_drone_time,'d{}'.format(drone_number))
                    #vprint('lauching_drone_event: ',lauching_drone_event)
                    solution['truck'].append(lauching_drone_event)
                    #vprint('truck_chrono before : ',truck_chrono)
                    truck_chrono=[truck_chrono[i] + preparing_drone_time if launched_drones[i] else truck_chrono[i] for i in range(self.number_of_drones)]
                    #vprint('truck_chrono after : ',truck_chrono )                    
                    launched_drones[drone_number-1]=True
                    #create drone go move
                    drone_go_move=(sp_truck[0],drone_target_list[drone_number-1],go_tt_list[drone_number-1])
                    solution['drone_{}'.format(drone_number)].append(drone_go_move)
                    #update demand
                    demand_dict[drone_target_list[drone_number-1]]-=1
                    #create drone back to truck move
                    drone_back_move=(drone_target_list[drone_number-1],truck_destination,back_tt_list[drone_number-1])
                    solution['drone_{}'.format(drone_number)].append(drone_back_move)
        #drone are launched or there is no drone to launch -> truck move toward its destination
        for node in sp_truck:
            if len(solution["truck"]) == 0:
                starting_node = self.instance.deposit
            else:
                starting_node = solution["truck"][-1][1]
            #create move event
            tt=round(nx.shortest_path_length(self.instance.graph,starting_node,node,weight='travel_time'),3)
            if tt!=0:
                move_event=(starting_node,node,tt)
                #vprint('move event : ',move_event)
                solution['truck'].append(move_event)
                truck_chrono=[truck_chrono[i] + tt if launched_drones[i] else truck_chrono[i] for i in range(self.number_of_drones)]
            if node==truck_destination:
                #vprint('-------DESTINATION NODE-------')
                if node in demand_dict.keys():
                    #vprint("demand : ", demand_dict[node])
                    #vprint('truck_chrono before delivery',truck_chrono)
                    #create delivery event
                    delivery_event=(node,node,delivering_truck_time)
                    #vprint('delivery event : ',delivery_event)
                    truck_chrono=[truck_chrono[i] + delivering_truck_time if launched_drones[i] else truck_chrono[i] for i in range(self.number_of_drones)]
                    solution['truck'].append(delivery_event)
                    #update demand
                    demand_dict[node]=0
                #deal with waiting time
                #split drone arrival into 2 list : before truck arrival and after truck arrival
                if not all(t==None for t in drone_target_list):
                    before_truck=[]
                    after_truck=[]
                    truck_chrono_for_compared=[]
                    drone_tt=[a+b for a,b in zip(go_tt_list,back_tt_list) if a!=None and b!=None]
                    #vprint('deal with waiting times')
                    #vprint('drone tt : ',drone_tt)
                    #vprint('truck_chrono : ',truck_chrono)
                    for time_t,time_d in zip(truck_chrono,drone_tt): 
                        if time_t !=None and time_d !=None and time_t<=time_d:
                            after_truck.append(time_d)
                            truck_chrono_for_compared.append(time_t)
                        elif time_t !=None and time_d !=None and time_t>time_d:
                            before_truck.append(time_d)
                    #vprint('before_truck : ',before_truck)
                    #vprint('after_truck : ',after_truck)
                    #vprint('truck_chrono : ',truck_chrono)
                    #vprint('drone_tt : ',drone_tt)
                    #case 1 -> drones wait for the truck
                    if len(before_truck)!=0:
                        #vprint('drone(s) wait for truck')
                        #compute waiting time for each drone
                        for time in before_truck:
                            #vprint('time : ',time)
                            drone_index=drone_tt.index(time)+1
                            #vprint('drone index : ',drone_index)
                            d_waiting_time=truck_chrono[drone_index-1]-time
                            #vprint('waiting time drone : ',d_waiting_time)
                            if d_waiting_time!=0:
                                if d_waiting_time<0:
                                    #vprint('drone waiting time : ',d_waiting_time)
                                    #vprint('truck chrono : ',truck_chrono)
                                    #vprint('drone tt time :',drone_tt)
                                    raise Exception('ERROR , drone waiting time <0')
                            else:
                                waiting_drone_event=(truck_destination,truck_destination,round(d_waiting_time,3))
                                #vprint('waiting_drone_{}_event : {}'.format(drone_index,waiting_drone_event))
                                #add it to the solution
                                solution['drone_{}'.format(drone_index)].append(waiting_drone_event) 
                    #case 2-> truck waits for drones 
                    if len(after_truck)!=0:
                            #vprint('truck waits for drones')
                            truck_waiting_time=max([d-t for d,t in zip(after_truck,truck_chrono_for_compared)])
                            #vprint('truck waiting time : ',truck_waiting_time)
                            if truck_waiting_time<0:
                                #vprint('truck waiting time : ',truck_waiting_time)
                                #vprint('truck chrono : ',truck_chrono)
                                #vprint('drone tt time :',drone_tt)
                                raise Exception('ERROR , drone waiting time <0')
                                
                            else:
                                waiting_truck_event=(truck_destination,truck_destination,round(truck_waiting_time,3))
                                #vprint('waiting_truck_event : ',waiting_truck_event)
                                solution['truck'].append(waiting_truck_event) 

    def improve_solution_2opt(self,solution, node_vehicle_dict):
        """improve the current solution"""
        vprint("===========IMPROVE SOLUTION 2 OPT==========")
        start_time=time.time()
        #idea 1 : change vehicles between two super nodes in the idea of 2-opt 
        self.init_node_vehicle_dict=node_vehicle_dict
        vprint('node_vehicle_dict : ',node_vehicle_dict)
        init_obj=self.calculate_obj_value(solution)
        vprint('init obj : ',init_obj)
        n=len(self.ordoned_demands_nodes)
        print('init tour :',self.ordoned_demands_nodes)
        vprint('init ordoner super node : ',self.ordoned_super_nodes)
        is_improved=False
        while True:
            best_obj=self.calculate_obj_value(solution)
            for i in range(1,n-2):
                for j in range(i+1,n):
                    new_tour=self.ordoned_demands_nodes[:]
                    new_tour[i:j]=self.ordoned_demands_nodes[j-1:i-1:-1]
                    #ordoned_super_nodes=[k for k in new_tour if k in self.super_nodes_dict.keys()]
                    #create solution from this tour
                    node_vehicle_dict=self.first_stage_v2(new_tour)
                    new_sol,is_feasible=self.second_stage_v2(node_vehicle_dict)
                    new_obj=self.calculate_obj_value(new_sol)
                    if new_obj<best_obj and is_feasible:
                        vprint('improvement founded')
                        best_tour=new_tour
                        #self.ordoned_demands_nodes=best_tour
                        best_obj=new_obj
                        best_sol=new_sol
                        best_node_vehicle_dict=node_vehicle_dict
                        is_improved=True
                        #print('----------------------')
                        #print('tour : ',best_tour)
                        #print('node vehicle dict : ',best_node_vehicle_dict)
                        #print('obj : ',best_obj)
                        #print('sol : ',best_sol)
                        break
                if new_obj<best_obj:
                    break
            if new_obj==best_obj:
                break
            if not is_improved:
                #best_tour=self
                best_tour=self.ordoned_demands_nodes
                best_sol=solution
                best_obj=init_obj
                best_node_vehicle_dict=self.init_node_vehicle_dict
            end_time=time.time()
            processing_time=end_time-start_time
            print('final tour : ',best_tour)
            self.ordoned_demands_nodes=best_tour
            self.node_vehicle_dict=best_node_vehicle_dict
            if solution==best_sol:
                print('best=init')
            vprint('processing improvment time : ',processing_time)
            return (best_sol,True,best_node_vehicle_dict,is_improved)
    
    def improve_affectation(self,solution,node_vehicle_dict):
        """improve the affectation node-vehicle for a given tour"""
        vprint('========IMPROVE AFFECTATION========')
        start_time=time.time()
        result=[]
        start=0
        vprint('ordoned demand nodes : ',self.ordoned_demands_nodes)
        for i,val in enumerate(self.ordoned_demands_nodes):
            if val in self.ordoned_super_nodes:
                result.append(self.ordoned_demands_nodes[start:i+1])
                start=i 
        result=[sublist for sublist in result if len(sublist)>1]
        vprint('result : ',result) 
        vprint('node vehicle dict : ',node_vehicle_dict) 
        print('longueur dict : ',len(node_vehicle_dict))
        #exchange the drone target 
        # Copier le dictionnaire original pour ne pas le modifier
        new_node_vehicle_dict = node_vehicle_dict.copy()
        best_obj=self.calculate_obj_value(solution)
        is_improved=False
        # Parcourir toutes les paires de clé-valeur dans le dictionnaire
        for i, (k, v) in enumerate(node_vehicle_dict.items()):
            # Si la valeur est 1 et la valeur suivante est 2, ou vice versa
            print('i = ',i)
            if i==len(node_vehicle_dict)-1:
                break
            print('list(node_vehicle_dict.values())[i+1]) : ',list(node_vehicle_dict.values())[i+1])
            if (v == 1 and list(node_vehicle_dict.values())[i+1] == 2) or \
            (v == 2 and list(node_vehicle_dict.values())[i+1] == 1):
                # Échanger les valeurs de clé pour les deux noeuds
                print('node vheicle before : ',node_vehicle_dict)
                new_node_vehicle_dict[k], new_node_vehicle_dict[list(node_vehicle_dict.keys())[i+1]] = \
                    new_node_vehicle_dict[list(node_vehicle_dict.keys())[i+1]], new_node_vehicle_dict[k]
                print('node vehicle after : ',new_node_vehicle_dict)
                #cree la solution a partir du dictionnaire
                new_sol,is_feasible=self.second_stage_v2(new_node_vehicle_dict)
                # Calculer le coût de la nouvelle solution
                new_obj=self.calculate_obj_value(new_sol)
                if new_obj<best_obj and is_feasible:
                    node_vehicle_dict = new_node_vehicle_dict.copy()
                    vprint('improvement founded')
                    #self.ordoned_demands_nodes=best_tour
                    best_obj=new_obj
                    best_sol=new_sol
                    best_node_vehicle_dict=node_vehicle_dict
                    is_improved=True
                # Sinon, revenir à la solution précédente
                else:
                    new_node_vehicle_dict[k], new_node_vehicle_dict[list(node_vehicle_dict.keys())[i+1]] = \
                        new_node_vehicle_dict[list(node_vehicle_dict.keys())[i+1]], new_node_vehicle_dict[k]
        if not is_improved:
            print('not improved')
            best_sol=solution
            best_obj=self.calculate_obj_value(solution)
            best_node_vehicle_dict=node_vehicle_dict
        # Renvoyer le dictionnaire modifié ou le dictionnaire original si aucune modification n'a été effectuée
        end_time=time.time()
        processing_time=end_time-start
        vprint('processing time : ',processing_time)
        print('best_node_vehicle_dict : ',best_node_vehicle_dict)
        print('best obj : ',best_obj)
        return (best_sol,True,best_node_vehicle_dict,is_improved)


    def calculate_obj_value(self,solution):
        """compute objective value for the heuristic solution"""
        truck_tour = solution["truck"]
        total_time = 0
        for move in truck_tour:
            total_time += move[2]
        return total_time

    def solve(self) -> VRPWDSolution:
        vprint("==============SOLVE================")
        start_time = time.time()
        node_vehicle_dict = self.first_stage_v2()
        solution,is_feasible=self.second_stage_v2(node_vehicle_dict)
        objective_value = self.calculate_obj_value(solution)
        vprint("initial objective value : ", objective_value)
        solution,is_feasible,best_node_vehicle_dict,is_improved=self.improve_affectation(solution,self.node_vehicle_dict)
        #solution,is_feasible,best_node_vehicle_dict,is_improved=self.improve_solution_2opt(solution,self.node_vehicle_dict)
        vprint('is final sol feasible ? :',is_feasible)
        if is_feasible:
            vprint('==========FINAL=========')
            print('final solution : ',solution)
            vprint('final objectif value : ',self.calculate_obj_value(solution))
            vprint('init node vehicle dict : ',self.node_vehicle_dict)
            print('final node vehicle dict : ',best_node_vehicle_dict)
            vprint('init has been improved ? : ',is_improved)
            end_time = time.time()
            processing_time = end_time - start_time
            vprint("processing time : {} s".format(processing_time))
            return VRPWDSolution(
                instance=self.instance,
                algorithm=self.__algorithm,
                objective_value=self.calculate_obj_value(solution),
                runtime=processing_time,
                gap="unknown",
                solution=solution,
                verbose=self.instance._VERBOSE,
            )
