from TSPWDData import TSPWDData
from pathlib import Path


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

    def check(self):
        # TODO: Check if the solution is feasible
        pass

    def write(self):
        # TODO:Write the solution to a file
        pass
