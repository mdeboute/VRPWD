"""
File to define edge object
"""


class Edge(object):
    # -----------------------------------------------
    def __init__(
        self,
        command_number,
        internal_id,
        from_node,
        to_node,
        length,
        speed,
        oneway,
        osm_id,
        osm_type,
    ):
        self.command_number = command_number
        self.internal_id = internal_id
        self.from_node = from_node
        self.to_node = to_node
        self.length = length
        self.speed = speed
        self.oneway = oneway
        self.osm_id = osm_id
        self.osm_type = osm_type

    # ---------------------------------------------------
    def get_command_number(self):
        return self.command_number

    # ---------------------------------------------------
    def get_internal_id(self):
        return self.internal_id

    # ---------------------------------------------------
    def get_from_node(self):
        return self.from_node

    # ---------------------------------------------------
    def get_to_node(self):
        return self.to_node

    # ---------------------------------------------------
    def get_length(self):
        return self.length

    # ---------------------------------------------------
    def get_speed(self):
        return self.speed

    # ---------------------------------------------------
    def get_oneway(self):
        return self.oneway

    # ---------------------------------------------------
    def get_osm_id(self):
        return self.osm_id

    # ---------------------------------------------------
    def get_osm_type(self):
        return self.osm_type
