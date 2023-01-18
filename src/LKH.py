from TSPWDData import TSPWDData


class LKH:
    """An implementation of the Lin-Kerninghan-Heldeger heuristic for the TSP."""

    def __init__(self, instance: TSPWDData):
        self.instance = instance
        self.__algorithm = "LKH"

    # TODO: Implement the Lin-Kerninghan-Heldeger heuristic
