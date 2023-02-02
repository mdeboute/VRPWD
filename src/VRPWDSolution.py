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
        if self.instance._CASE == 0:
            # check if all demand nodes are in the tour
            for node in self.instance.dpd_nodes[1:]:
                if node not in [tuple[0] for tuple in list(self.solution["truck"])]:
                    print(f"ERROR: demand node {node} is not in the tour!")
                    return False
            # check that we start and end at the deposit
            if self.solution["truck"][0][0] != self.instance.deposit:
                print("ERROR: tour does not start at the deposit!")
                return False
            if self.solution["truck"][-1][0] != self.instance.deposit:
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
        # coords = nx.get_node_attributes(self.graph, "coordinates")

        print(truck_shift)

        # with open(_sol_file, "w") as f:
        #     f.write("TEMPS ; EVENEMENT ; LOCALISATION")

        pass
