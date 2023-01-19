class Edge(object):
    def __init__(
        self,
        index: int,
        source: int,
        destination: int,
        length: float,
        speed: int,
        osm_id: int,
        osm_type: str,
        travel_time: float,
    ):
        self.index = index
        self.source = source
        self.destination = destination
        self.length = length
        self.speed = speed
        self.osm_id = osm_id
        self.osm_type = osm_type
        self.travel_time = travel_time
