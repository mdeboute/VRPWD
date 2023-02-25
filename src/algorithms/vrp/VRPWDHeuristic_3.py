import time
import numpy as np

from core.VRPWDData import VRPWDData
from core.VRPWDSolution import VRPWDSolution
from algorithms.vrp.VRPWDPathHeuristic_2 import VRPWDPathHeuristic_2
from core.utils import verbose_print
from math import atan2, cos, sin, sqrt, radians
from scipy.spatial.distance import squareform
from scipy.cluster.hierarchy import linkage, fcluster


class VRPWDHeuristic_3:
    def __init__(self, instance: VRPWDData):
        self.instance = instance
        self.__algorithm = "Cluster_Greedy"
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

    def _get_clusters(self):
        demand_nodes = self.instance.dpd_nodes[1:]
        matrix = self.instance.dpd_time_matrix[1:, 1:]
        # Compute the distance matrix between demand nodes
        dist_matrix = np.maximum(matrix, matrix.T)
        # Apply hierarchical clustering to the distance matrix
        Z = linkage(squareform(dist_matrix), method="ward")
        # Set the desired number of clusters based on the dendrogram or elbow method
        num_clusters = 3
        # Get the cluster assignments for each demand node
        cluster_assignments = fcluster(Z, num_clusters, criterion="maxclust")
        # Create a list of partitions
        partitions = [[] for _ in range(num_clusters)]
        for i, demand_node in enumerate(demand_nodes):
            partitions[cluster_assignments[i] - 1].append(demand_node)
        return partitions

    def _get_centroid(self, cluster):
        cluster_coords = [self.graph.nodes[node]["coordinates"] for node in cluster]
        cluster_centroid = np.mean(cluster_coords, axis=0)
        return cluster_centroid

    def _find_nearest_move_to_centroid(self, centroid, subtour_sol):
        # i want to find the nearest move (i.e the tuple (i, j, ...)) to the centroid
        # i.e the move that minimizes the distance between the centroid and the move

        # get visited nodes
        visited_nodes = []
        for move in subtour_sol:
            visited_nodes.append(move[0])
        visited_nodes = list(set(visited_nodes))
        vprint(visited_nodes)

        # find the nearest node to the centroid
        nearest_node = min(
            visited_nodes,
            key=lambda x: self._haversine(
                *centroid, *self.graph.nodes[x]["coordinates"]
            ),
        )
        return nearest_node

    def solve(self):
        start_time = time.time()

        # find a cluster
        clusters = self._get_clusters()
        # select the cluster with the lowest total demand
        cluster = min(clusters, key=lambda x: sum(self.demands_nodes[i] for i in x))
        vprint(cluster)

        # make the demand nodes in the cluster to 0 in the graph
        for node in cluster:
            self.instance.graph.nodes[node]["demand"] = 0
        subtour_sol = VRPWDPathHeuristic_2(self.instance).solve()
        subtour = subtour_sol.solution["truck"]

        centroid = self._get_centroid(cluster)
        nearest_node = self._find_nearest_move_to_centroid(centroid, subtour)
        vprint(nearest_node)
        # find the first occurence of the nearest node in the subtour
        first_occurence_idx = next(
            i for i, x in enumerate(subtour) if x[0] == nearest_node
        )
        # TODO: to be continued
        end_time = time.time()
        runtime = end_time - start_time
        pass
