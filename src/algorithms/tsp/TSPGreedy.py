import numpy as np
import time

from core.VRPWDData import VRPWDData
from core.VRPWDSolution import VRPWDSolution
from algorithms.tsp.TSPMIPModel import create_solution


class TSPGreedy:
    """An implementation of the Greedy heuristic for the TSP."""

    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Nearest_Neighbor_Greedy"

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
        objective_value = sum(
            solution["truck"][i][2] for i in range(len(solution["truck"]))
        )

        end = time.time()
        runtime = end - start

        # Return the solution
        return VRPWDSolution(
            instance=self.instance,
            algorithm=self.__algorithm,
            objective_value=round(objective_value),
            runtime=runtime,
            gap="unknown",
            solution=solution,
            verbose=self.instance._VERBOSE,
        )
