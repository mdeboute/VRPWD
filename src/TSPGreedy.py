from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
import numpy as np
import time


class TSPGreedy:
    """An implementation of the Greedy heuristic for the TSP."""

    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "NN_Greedy"

    def solve(self) -> VRPWDSolution:
        """Solve the TSP using the Greedy heuristic."""

        start = time.time()
        # Initialize the solution
        solution = [0]
        unvisited = np.array(range(1, self.instance.dpd_time_matrix.shape[0]))

        # While there are unvisited cities
        while unvisited.size != 0:
            # Find the closest unvisited city
            last_city = solution[-1]
            closest_city = unvisited[
                np.argmin(self.instance.dpd_time_matrix[last_city, unvisited])
            ]

            # Add it to the solution
            solution.append(closest_city)
            unvisited = np.delete(unvisited, np.where(unvisited == closest_city))

        # Add the return to the deposit
        solution.append(0)

        # Compute the ojective value, i.e. the total time to visit all cities
        obj_value = np.sum(self.instance.dpd_time_matrix[solution[:-1], solution[1:]])

        # Update solution to have the id of the nodes thanks to instance.demands_nodes
        solution = [self.instance.dpd_nodes[i] for i in solution]
        end = time.time()
        print(
            "Time to solve the TSP with Greedy heuristic: ",
            round(end - start, 4),
            "sec",
        )
        # Return the solution
        return VRPWDSolution(
            self.instance,
            self.__algorithm,
            round(obj_value),
            solution,
            self.instance._VERBOSE,
        )
