class Node(object):
    def __init__(self, lat: float, lon: float):
        assert isinstance(lat, float), "ERROR: lat must be a float!"
        assert isinstance(lon, float), "ERROR: lon must be float!"
        self.lat = lat
        self.lon = lon
