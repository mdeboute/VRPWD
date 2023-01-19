from TSPWDData import TSPWDData
from TSPWDSolution import TSPWDSolution


class TSPGreedy:
    """An implementation of the Greedy heuristic for the TSP."""

    def __init__(self, instance: TSPWDData):
        self.instance = instance
        self.__algorithm = "Nearest Neighbour Greedy"

    def solve(self) -> TSPWDSolution:
        """Solve the TSP using the Greedy heuristic."""

        # Initialize the solution
        solution = [0]
        unvisited = set(range(1, self.instance.time_matrix.shape[0]))

        # While there are unvisited cities
        while unvisited:
            # Find the closest unvisited city
            last_city = solution[-1]
            closest_city = min(
                unvisited, key=lambda c: self.instance.time_matrix[last_city, c]
            )

            # Add it to the solution
            solution.append(closest_city)
            unvisited.remove(closest_city)

        # Add the return to the depot
        solution.append(0)

        # Compute the ojective value, i.e. the total time to visit all cities
        obj_value = sum(
            self.instance.time_matrix[solution[i], solution[i + 1]]
            for i in range(len(solution) - 1)
        )

        # Update solution to have the id of the nodes thanks to instance.demands_nodes
        solution = [self.instance.demands_nodes[i] for i in solution]

        # Return the solution
        return TSPWDSolution(
            self.instance,
            self.__algorithm,
            round(obj_value),
            solution,
        )
