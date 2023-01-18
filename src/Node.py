class Node(object):
    def __init__(self, index: int, lat: float, lon: float, demand: float):
        assert isinstance(index, int), "ERROR: idx must be an int!"
        assert isinstance(lat, float), "ERROR: lat must be a float!"
        assert isinstance(lon, float), "ERROR: lon must be a float!"
        assert isinstance(demand, float), "ERROR: lon must be a float!"
        self.index=index
        self.lat = lat
        self.lon = lon
        self.demand = demand
