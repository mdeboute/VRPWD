import numpy as np
import time

from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from TSPMIPModel import create_solution


class TSPGreedy:
    """An implementation of the Greedy heuristic for the TSP."""

    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "NN_Greedy"

    def solve(self) -> VRPWDSolution:
        """Solve the TSP using the Greedy heuristic."""

        start = time.time()
        # Initialize the solution
        tour = [0]
        unvisited = np.array(range(1, self.instance.dpd_time_matrix.shape[0]))

        # While there are unvisited cities
        while unvisited.size != 0:
            # Find the closest unvisited city
            last_city = tour[-1]
            closest_city = unvisited[
                np.argmin(self.instance.dpd_time_matrix[last_city, unvisited])
            ]

            # Add it to the solution
            tour.append(closest_city)
            unvisited = np.delete(unvisited, np.where(unvisited == closest_city))

        # Add the return to the deposit
        tour.append(0)

        solution = create_solution(self.instance, tour)
        obj_value = solution["truck"][-1][-1]

        end = time.time()
        print(f"Result: runtime={end-start:.2f}sec; objective={obj_value}")

        # Return the solution
        return VRPWDSolution(
            self.instance,
            self.__algorithm,
            round(obj_value),
            solution,
            self.instance._VERBOSE,
        )
