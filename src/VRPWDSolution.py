from VRPWDData import VRPWDData
from pathlib import Path
import networkx as nx
import matplotlib.pyplot as plt
from utils import verbose_print


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
        pass

    def plot(self):
        pass

    def check(self):
        pass

    def write(self):
        Path(self.__SOLUTION_DIR).mkdir(parents=True, exist_ok=True)

        _sol_file = self.__SOLUTION_DIR + self.algorithm + "_result.txt"

        with open(_sol_file, "w") as f:
            pass
        pass


# TODO: implement this class
