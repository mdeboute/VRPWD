class Edge(object):
    def __init__(
        self,
        index,
        source,
        destination,
        length,
        speed,
        osm_id,
        osm_type,
        travel_time
    ):
        self.index = index
        self.source = source
        self.destination = destination
        self.length = length
        self.speed = speed
        self.osm_id = osm_id
        self.osm_type = osm_type
        self.travel_time=travel_time
