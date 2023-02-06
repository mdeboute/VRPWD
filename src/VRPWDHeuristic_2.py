from VRPWDData import VRPWDData
from TSPMIPModel import TSPMIPModel
from utils import verbose_print

class VRPWDHeuristic_2:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.init_sol = TSPMIPModel(self.instance)

    def first_stage(self):
        """ first stage part of the heuristic
        -given the TSP solution, affect the k drones to the k first nodes
        while the truck goes to the k+1th node etc"""
        print(self.init_sol)

    def seconde_stage(self):
        pass

    def solve(self):
        self.first_stage()
