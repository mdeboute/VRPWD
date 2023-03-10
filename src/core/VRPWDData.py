import pandas as pd
import geopandas as gpd
import numpy as np
import networkx as nx
import folium
import time

from pathlib import Path
from scipy.spatial import cKDTree
from geopy.distance import geodesic
from core.utils import verbose_print


class VRPWDData:
    """This class is used to store the instance information of the VRPWD problem."""

    def __init__(self, instance_dir: str, case: int, verbose: bool):
        self.__MAP_PATH = Path(instance_dir).joinpath("map.json")
        self.__DEMANDS_PATH = Path(instance_dir).joinpath("demands.json")

        self._INSTANCE_NAME = instance_dir.split("/")[-1]
        self._CASE = case
        self._VERBOSE = verbose
        global vprint
        vprint = verbose_print(self._VERBOSE)

        self.__brut_df_map = pd.read_json(self.__MAP_PATH)
        self.__brut_df_demands = pd.read_json(self.__DEMANDS_PATH)
        self.__gdf_nodes, self.__gdf_edges = self._create_gdfs()

        self.graph = self._create_graph()
        self.dpd_time_matrix = self._create_dpd_time_matrix()

        if self._CASE > 0:  # vrp with drones
            self.drone_time_matrix = self._create_drone_time_matrix(drone_speed=50)

    def __str__(self) -> str:
        return f"Instance: {self._INSTANCE_NAME}, Case: {self._CASE}"

    def __repr__(self) -> str:
        return self.__str__()

    def _create_gdfs(self):
        start_time = time.time()
        vprint("=================== NODES AND EDGES GDF CREATION ===================")
        coords_dict = {}
        rows_gdf_nodes = []
        cols_gdf_nodes = ["lat", "lon", "demand"]
        rows_gdf_edges = []
        cols_gdf_edges = [
            "src",
            "dest",
            "length",
            "speed",
            "osm_id",
            "osm_type",
            "travel_time",
        ]
        for _, row in self.__brut_df_map.iterrows():
            if (row["lat_min"], row["lon_min"]) not in coords_dict:
                coords_dict[(row["lat_min"], row["lon_min"])] = len(coords_dict) + 1
            idx_src = coords_dict[(row["lat_min"], row["lon_min"])]
            if (row["lat_max"], row["lon_max"]) not in coords_dict:
                coords_dict[(row["lat_max"], row["lon_max"])] = len(coords_dict) + 1
            idx_dest = coords_dict[(row["lat_max"], row["lon_max"])]
            length = row["length"]
            osm_id = row["osmid"]
            osm_type = row["type"]
            if osm_type == "primary":
                speed = 60
            elif osm_type == "secondary":
                speed = 45
            else:
                speed = 30
            m_per_s_speed = round(speed / 3.6, 2)
            travel_time = length / m_per_s_speed
            rows_gdf_edges.append(
                {
                    "src": idx_src,
                    "dest": idx_dest,
                    "length": length,
                    "speed": speed,
                    "osm_id": osm_id,
                    "osm_type": osm_type,
                    "travel_time": travel_time,
                }
            )
        for coord, _ in coords_dict.items():
            x = coord[0]
            y = coord[1]
            rows_gdf_nodes.append({"lat": x, "lon": y, "demand": 0})
        gdf_nodes = gpd.GeoDataFrame(rows_gdf_nodes, columns=cols_gdf_nodes)
        gdf_edges = gpd.GeoDataFrame(rows_gdf_edges, columns=cols_gdf_edges)
        end_time = time.time()
        processing_time = end_time - start_time
        vprint(gdf_nodes)
        vprint(gdf_edges)
        vprint("processing_time:", processing_time)
        return gdf_nodes, gdf_edges

    def _create_graph(self):
        vprint("==================== GRAPH CREATION ====================")
        start_time = time.time()
        # create empty undirected graph
        graph = nx.Graph()
        # add nodes
        for idx, row in self.__gdf_nodes.iterrows():
            lat = row["lat"]
            lon = row["lon"]
            coord = (lat, lon)
            d = row["demand"]
            graph.add_node(idx + 1, coordinates=coord, deposit=False, demand=d)
        # add edges
        for idx, row in self.__gdf_edges.iterrows():
            src = row["src"]
            dest = row["dest"]
            length = row["length"]
            speed = row["speed"]
            osm_type = row["osm_type"]
            osm_id = row["osm_id"]
            travel_time = row["travel_time"]
            graph.add_edge(
                src,
                dest,
                id=idx + 1,
                length=length,
                osm_type=osm_type,
                speed=speed,
                osm_id=osm_id,
                travel_time=travel_time,
            )
        # Build K-D Tree
        coords = self.__gdf_nodes[["lat", "lon"]].to_numpy()
        tree = cKDTree(coords)
        # add demand
        for _, row in self.__brut_df_demands.iterrows():
            point = (row["lat"], row["lon"])
            demand = row["amount"]
            # Find nearest node with K-D Tree
            _, nearest_node = tree.query(point)
            nearest_node = nearest_node + 1
            # update the demand in the graph
            graph.nodes[nearest_node].update({"demand": demand})
            # in the df
            self.__gdf_nodes.loc[nearest_node, "demand"] = demand
        # add deposit
        _deposit_gps = (44.8500102, 0.5370699)
        # Find nearest node with K-D Tree
        _, nearest_node = tree.query(_deposit_gps)
        nearest_node = nearest_node + 1
        # update the deposit in the Data object
        self.deposit = nearest_node
        # in the graph
        graph.nodes[self.deposit].update({"deposit": True})
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("graph:", graph)
        vprint("processing_time:", processing_time)
        return graph

    def _create_dpd_time_matrix(self):
        """Create the D+1xD+1 travel time matrix from the road point of view
        with D the number of demand nodes, +1 for the deposit.
        By default, the deposit is the first node of the list 'demand_nodes'"""

        vprint("================== CREATE TIME MATRIX ==================")
        start_time = time.time()
        demands_nodes = [
            node for node in self.graph.nodes() if self.graph.nodes[node]["demand"] > 0
        ]
        vprint("deposit:", self.deposit)
        # add deposit to the list of demand_nodes in order to calculate travel_time between demand nodes and deposit
        # add it at the beginning of the list
        demands_nodes.insert(0, self.deposit)
        self.dpd_nodes = demands_nodes
        vprint("dpd_nodes:", self.dpd_nodes)
        # create empty matrix
        matrix = np.zeros(shape=(len(demands_nodes), len(demands_nodes)), dtype=float)
        vprint("matrix_shape:", matrix.shape)
        # pre-compute shortest paths
        shortest_paths = dict(
            nx.all_pairs_dijkstra_path_length(self.graph, weight="travel_time")
        )
        # fill matrix using list comprehension
        matrix[np.arange(len(demands_nodes)), np.arange(len(demands_nodes))] = 0
        for i in range(len(demands_nodes)):
            for j in range(i + 1, len(demands_nodes)):
                matrix[i][j] = matrix[j][i] = round(
                    shortest_paths[demands_nodes[i]][demands_nodes[j]], 3
                )
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time:", processing_time)
        vprint("matrix:", matrix)
        return matrix

    def _create_drone_time_matrix(self, drone_speed):
        """Create the time travel matrix from the drone point of view"""

        vprint("================ CREATE DRONE MATRIX ================")
        start_time = time.time()
        number_of_nodes = self.graph.number_of_nodes()
        matrix = np.zeros(shape=(number_of_nodes, number_of_nodes), dtype=float)
        vprint("matrix_shape:", matrix.shape)
        coordinates = {
            node: (
                self.graph.nodes[node]["coordinates"][1],
                self.graph.nodes[node]["coordinates"][0],
            )
            for node in self.graph.nodes
        }
        nodes = list(self.graph.nodes)
        for i in range(number_of_nodes):
            current_node = nodes[i]
            current_node_coord = coordinates[current_node]
            for j in range(i + 1, number_of_nodes):
                other_node = nodes[j]
                other_node_coord = coordinates[other_node]
                dist = geodesic(current_node_coord, other_node_coord).m
                m_per_s_drone_speed = drone_speed / 3.6
                travel_time = round(dist / m_per_s_drone_speed, 3)
                matrix[current_node - 1][other_node - 1] = travel_time
                matrix[other_node - 1][current_node - 1] = travel_time
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time:", processing_time)
        vprint("matrix:", matrix)
        return matrix

    def save_map_html(self):
        """Plot the nodes on an interactive html map"""

        # Create map
        m = folium.Map(location=[44.838633, 0.540983], zoom_start=13)
        # Add points to the map according to the demand
        for node in self.graph.nodes():
            coord = self.graph.nodes[node]["coordinates"]
            if self.graph.nodes[node]["deposit"]:
                folium.Marker(
                    coord,
                    popup="Deposit",
                    icon=folium.Icon(color="green"),
                    tooltip=f"{node}",
                ).add_to(m)
            elif self.graph.nodes[node]["demand"] > 0:
                # add the demand value as a popup
                folium.Marker(
                    coord,
                    popup=f"Demand: {int(self.graph.nodes[node]['demand'])}",
                    icon=folium.Icon(color="red"),
                    tooltip=f"{node}",
                ).add_to(m)
            else:
                folium.Marker(
                    coord,
                    icon=folium.Icon(color="blue"),
                    tooltip=f"{node}",
                ).add_to(m)
        # save the map in a html file
        _assets_path = "assets/" + self._INSTANCE_NAME
        Path(_assets_path).mkdir(parents=True, exist_ok=True)
        m.save(_assets_path + "/map.html")
        print(f"HTML Map saved in {_assets_path}/map.html")
