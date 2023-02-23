import networkx as nx
import time

from VRPWDData import VRPWDData
from VRPWDSolution import VRPWDSolution
from VRPWDPathHeuristic_2 import VRPWDPathHeuristic_2
from utils import verbose_print
from math import atan2, cos, sin, sqrt, radians


class VRPWDHeuristic_3:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Greedy"
        self.init_sol = VRPWDPathHeuristic_2(self.instance).solve()
        self.demands_nodes = {
            node: int(self.instance.graph.nodes[node]["demand"])
            for node in self.instance.dpd_nodes[1:]
        }
        self.graph = self.instance.graph.copy()

        global vprint
        vprint = verbose_print(self.instance._VERBOSE)

    def _haversine(self, lat1, lon1, lat2, lon2):
        R = 6371.0
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c

    def _find_closest_gps_point_on_segment(self, p1, p2, p3):
        coords_p1 = p1["coordinates"]
        coords_p2 = p2["coordinates"]
        coords_p3 = p3["coordinates"]
        x1, y1 = coords_p1[0], coords_p1[1]
        x2, y2 = coords_p2[0], coords_p2[1]
        x3, y3 = coords_p3[0], coords_p3[1]
        dist_p1p3 = self._haversine(x1, y1, x3, y3)
        dist_p2p3 = self._haversine(x2, y2, x3, y3)
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            return {"lat": x1, "lon": y1}
        t = ((x3 - x1) * dx + (y3 - y1) * dy) / (dx * dx + dy * dy)
        t = max(0, min(1, t))
        x, y = x1 + t * dx, y1 + t * dy
        dist_new_point_p3 = self._haversine(x, y, x3, y3)
        if dist_new_point_p3 <= dist_p1p3 and dist_new_point_p3 <= dist_p2p3:
            return {"lat": x, "lon": y}
        elif dist_p1p3 < dist_p2p3:
            return {"lat": x1, "lon": y1}
        else:
            return {"lat": x2, "lon": y2}

    def solve(self):
        print(self.graph.nodes[114])
        print(self.graph.nodes[116])
        print(self.graph.nodes[115])
        print(
            self._find_closest_gps_point_on_segment(
                self.graph.nodes[114],
                self.graph.nodes[116],
                self.graph.nodes[115],
            )
        )  # see the map
