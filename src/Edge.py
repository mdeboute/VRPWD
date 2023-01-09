class Edge(object):
    def __init__(
        self,
        command_number,
        internal_id,
        source,
        destination,
        length,
        speed,
        oneway,
        osm_id,
        osm_type,
    ):
        self.command_number = command_number
        self.internal_id = internal_id
        self.source = source
        self.destination = destination
        self.length = length
        self.speed = speed
        self.oneway = oneway
        self.osm_id = osm_id
        self.osm_type = osm_type
