import networkx as nx
import time

from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from VRPWDPathHeuristic_2 import VRPWDPathHeuristic_2
from utils import verbose_print


class VRPWDHeuristic_1:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Greedy?"
        self.init_sol = VRPWDPathHeuristic_2(self.instance).solve()
        self.demands_nodes = {
            node: int(self.instance.graph.nodes[node]["demand"])
            for node in self.instance.dpd_nodes[1:]
        }
        self.graph = self.instance.graph.copy()

        global vprint
        vprint = verbose_print(self.instance._VERBOSE)
