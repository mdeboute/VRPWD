"""
File to define a node object
"""


class Node(object):
    # -----------------------------------------------
    def __init__(self, lat, lon):
        assert isinstance(lat, float), "ERROR: lat must be a float!"
        assert isinstance(lon, float), "ERROR: lon must be float!"
        self.lat = lat
        self.lon = lon

    # -----------------------------------------------
    def get_lat(self):
        return self.lat

    # -----------------------------------------------
    def get_lon(self):
        return self.lon
