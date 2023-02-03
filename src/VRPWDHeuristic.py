from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from TSPGreedy import TSPGreedy


class VRPWDHeuristic:
    def __init__(self, instance: VRPWDData):
        self.instance = instance

    def solve(self) -> VRPWDSolution:
        init_sol = TSPGreedy(self.instance).solve()
