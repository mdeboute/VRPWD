class Node(object):
    def __init__(self, lat: float, lon: float, demand: int):
        assert isinstance(lat, float), "ERROR: lat must be a float!"
        assert isinstance(lon, float), "ERROR: lon must be a float!"
        assert isinstance(lon, float), "ERROR: demand must be an int!"
        self.lat = lat
        self.lon = lon
        self.demand = demand
