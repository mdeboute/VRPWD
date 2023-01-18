import mip
from TSPWDData import TSPWDData
from TSPWDSolution import TSPWDSolution


class FLPMIPModel:
    def __init__(self, instance: TSPWDData):
        self.instance = instance
        self.__algorithm = "MIP"
        self.model = mip.Model(name="TSP", sense=mip.MINIMIZE)
        pass
        # TODO: Create the mip model
