from TSPWDData import TSPWDData
from TSPWDSolution import TSPWDSolution
import numpy as np


class TSPGreedy:
    """An implementation of the Greedy heuristic for the TSP."""

    def __init__(self, instance: TSPWDData):
        self.instance = instance
        self.__algorithm = "Nearest Neighbour Greedy"

    def solve(self) -> TSPWDSolution:
        """Solve the TSP using the Greedy heuristic."""

        # Initialize the solution
        solution = [0]
        unvisited = np.array(range(1, self.instance.time_matrix.shape[0]))

        # While there are unvisited cities
        while unvisited.size != 0:
            # Find the closest unvisited city
            last_city = solution[-1]
            closest_city = unvisited[
                np.argmin(self.instance.time_matrix[last_city, unvisited])
            ]

            # Add it to the solution
            solution.append(closest_city)
            unvisited = np.delete(unvisited, np.where(unvisited == closest_city))

        # Add the return to the depot
        solution.append(0)

        # Compute the ojective value, i.e. the total time to visit all cities
        obj_value = np.sum(self.instance.time_matrix[solution[:-1], solution[1:]])

        # Update solution to have the id of the nodes thanks to instance.demands_nodes
        solution = [self.instance.demands_nodes[i] for i in solution]

        # Return the solution
        return TSPWDSolution(
            self.instance,
            self.__algorithm,
            round(obj_value),
            solution,
        )
