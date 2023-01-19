import pandas as pd
import geopandas as gpd
import random
import numpy as np
import matplotlib.pyplot as plt
from geopy.distance import geodesic
import time
import networkx as nx
import folium
from pathlib import Path
from Node import Node
from Edge import Edge


class TSPWDData(object):
    """This class is used to store the instance information"""

    def __init__(self, instance_dir, case, verbose):
        self.__MAP_PATH = Path(instance_dir).joinpath("map.json")
        self.__DEMANDS_PATH = Path(instance_dir).joinpath("demands.json")
        self.__CASE = case
        self.__VERBOSE = verbose

        if self.__VERBOSE:

            def _vprint(*args, **kwargs):
                print(*args, **kwargs)

        else:
            _vprint = lambda *_, **__: None  # do-nothing function

        global vprint
        vprint = _vprint

        self._brut_df_map = pd.read_json(self.__MAP_PATH)
        self._brut_df_demands = pd.read_json(self.__DEMANDS_PATH)

        self.nodes, self.edges = self._create_nodes_and_edges_df()
        self.graph = self._create_graph()
        self.df_node_objects = self._create_df_node_objects()
        self.df_edge_objects = self._create_df_edge_objects()
        self.time_matrix = self._create_time_matrix()
        if self.__CASE > 0:
            self.drone_matrix = self._create_drone_matrix(drone_speed=50)

    def _create_nodes_and_edges_df(self):
        start_time = time.time()
        vprint("=================== BRUT NODES AND EDGES CREATION ===================")
        list_coords = []
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

        for _, row in self._brut_df_map.iterrows():
            if (row["lat_min"], row["lon_min"]) in list_coords:
                idx_src = list_coords.index((row["lat_min"], row["lon_min"])) + 1
            else:
                list_coords.append((row["lat_min"], row["lon_min"]))
                idx_src = len(list_coords)
            if (row["lat_max"], row["lon_max"]) in list_coords:
                idx_dest = list_coords.index((row["lat_max"], row["lon_max"])) + 1
            else:
                list_coords.append((row["lat_max"], row["lon_max"]))
                idx_dest = len(list_coords)

            length = row["length"]
            osm_id = row["osmid"]
            osm_type = row["type"]
            if osm_type == "primary":
                speed = 60
            if osm_type == "secondary":
                speed = 45
            if osm_type == "tertiary":
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
        gdf_edges = gpd.GeoDataFrame(rows_gdf_edges, columns=cols_gdf_edges)
        gdf_edges.index = range(1, len(gdf_edges) + 1)
        for coord in list_coords:
            x = coord[0]
            y = coord[1]
            rows_gdf_nodes.append({"lat": x, "lon": y, "demand": 0})
        gdf_nodes = gpd.GeoDataFrame(rows_gdf_nodes, columns=cols_gdf_nodes)
        # there is duplicate node if brut_gdf_nodes, so we drop them
        gdf_nodes = gdf_nodes.drop_duplicates(subset=["lat", "lon"], keep="first")
        # reindex the gdf
        gdf_nodes.index = range(1, len(gdf_nodes) + 1)
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time = ", processing_time)
        return gdf_nodes, gdf_edges

    def _create_graph(self):
        vprint("==================== GRAPH CREATION ====================")
        start_time = time.time()
        # create empty undirected graph
        graph = nx.Graph()
        # add nodes
        for idx, row in self.nodes.iterrows():
            lat = row["lat"]
            lon = row["lon"]
            coord = (lon, lat)
            d = row["demand"]
            graph.add_node(idx, coordinates=coord, demand=d)
        # add edges
        for idx, row in self.edges.iterrows():
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
                id=idx,
                length=length,
                osm_type=osm_type,
                speed=speed,
                osm_id=osm_id,
                travel_time=travel_time,
            )
        # add demand
        for _, row in self._brut_df_demands.iterrows():
            point = (row["lat"], row["lon"])
            demand = row["amount"]
            nearest_node = None
            nearest_distance = float("inf")
            for node in graph.nodes:
                node_coord = graph.nodes[node]["coordinates"]
                node_coords_inversed = (node_coord[1], node_coord[0])
                # compute euclidian distance through Haversine formula
                dist = geodesic(point, node_coords_inversed).km
                if dist < nearest_distance:
                    nearest_distance = dist
                    nearest_node = node
            # update the demand
            # in the graph
            graph.nodes[nearest_node].update({"demand": demand})
            # in the df
            self.nodes.loc[nearest_node, "demand"] = demand
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time = ", processing_time)
        return graph

    def _create_df_node_objects(self):
        """create a df with 1 column 'Node Object' with all nodes"""
        vprint("=========== DF NODE OBJECTS CREATION ===========")
        start_time = time.time()
        rows_df_nodes = []
        cols_df_nodes = ["Node Object"]
        for idx, row in self.nodes.iterrows():
            node = Node(idx, row["lat"], row["lon"], row["demand"])
            rows_df_nodes.append({"Node Object": node})
        df_nodes_object = pd.DataFrame(
            rows_df_nodes, columns=cols_df_nodes, index=range(1, len(self.nodes) + 1)
        )
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time = ", processing_time)
        return df_nodes_object

    def _create_df_edge_objects(self):
        """create a df with 1 column 'Edge Object' with all edges"""
        vprint("=========== DF EDGE OBJECTS CREATION ===========")
        start_time = time.time()
        rows_df_edges = []
        cols_df_edges = ["Edge Object"]
        vprint("nb_edges_in_brut_df = ", len(self.edges))
        vprint("nb_edges_in_graph = ", self.graph.number_of_edges())
        i = 1
        for e in self.graph.edges():
            edge = Edge(
                i,
                e[0],
                e[1],
                self.graph.edges[e]["length"],
                self.graph.edges[e]["speed"],
                self.graph.edges[e]["osm_id"],
                self.graph.edges[e]["osm_type"],
                self.graph.edges[e]["travel_time"],
            )
            rows_df_edges.append({"Edge Object": edge})
        df_edge_objects = pd.DataFrame(
            rows_df_edges,
            columns=cols_df_edges,
            index=range(1, self.graph.number_of_edges() + 1),
        )
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time = ", processing_time)
        return df_edge_objects

    def _create_time_matrix(self):
        """Create the D+1xD+1 travel time matrix from the road point of view
        with D the number of demand nodes, +1 for the depot.
        By default, the depot is the first node of the list 'demand_nodes'"""

        vprint("================== CREATE TIME MATRIX ==================")
        start_time = time.time()
        round_precision = 3
        demands_nodes = []
        depot = None
        # get demand nodes
        for node in self.graph.nodes():
            if self.graph.nodes[node]["demand"] > 0:
                demands_nodes.append(node)
        vprint("demand_nodes = ", demands_nodes)
        # select a random nodes different from those with demand>0
        while (depot == None) or (depot in demands_nodes):
            depot = random.randint(1, self.graph.number_of_nodes())
        vprint("depot = ", depot)
        # add depot to the list of demand_nodes in order to calculate travel_time between demand nodes and depot
        # add it at the beginning of the list
        demands_nodes.insert(0, depot)

        vprint("demand_nodes_and_depot = ", demands_nodes)
        self.demands_nodes = demands_nodes
        # create empty matrix
        matrix = np.zeros(shape=(len(demands_nodes), len(demands_nodes)), dtype=float)
        vprint("matrix_shape = ", matrix.shape)
        # compute shortest path in term of travel time
        for current_node in demands_nodes:
            for other_node in demands_nodes:
                if current_node != other_node:
                    travel_time = round(
                        nx.dijkstra_path_length(
                            self.graph, current_node, other_node, weight="travel_time"
                        ),
                        round_precision,
                    )
                    matrix[demands_nodes.index(current_node)][
                        demands_nodes.index(other_node)
                    ] = travel_time
                    matrix[demands_nodes.index(other_node)][
                        demands_nodes.index(current_node)
                    ] = travel_time
        for i in range(len(matrix)):
            matrix[i][i] = 10000
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time = ", processing_time)
        return matrix

    def _create_drone_matrix(self, drone_speed):
        """Create the time travel matrix from the drone point of view"""

        vprint("================ CREATE DRONE MATRIX ================")
        start_time = time.time()
        round_number = 3
        # create matrix of dimension NxN with N the number of nodes in the graph
        number_of_nodes = self.graph.number_of_nodes()
        vprint("number_of_nodes = ", number_of_nodes)
        matrix = np.zeros(shape=(number_of_nodes, number_of_nodes), dtype=float)
        vprint("matrix shape = ", matrix.shape)
        # loop over nodes to calculate travel time between current node and others nodes
        for current_node in self.graph.nodes:
            for other_node in self.graph.nodes:
                if current_node != other_node:
                    if matrix[current_node - 1][other_node - 1] == 0:
                        # get coordinates for both nodes
                        current_node_coord = self.graph.nodes[current_node][
                            "coordinates"
                        ]
                        current_node_coords_inversed = (
                            current_node_coord[1],
                            current_node_coord[0],
                        )
                        other_node_coord = self.graph.nodes[other_node]["coordinates"]
                        other_node_coord_inversed = (
                            other_node_coord[1],
                            other_node_coord[0],
                        )
                        dist = geodesic(
                            current_node_coords_inversed, other_node_coord_inversed
                        ).m
                        # calculate travel time
                        m_per_s_drone_speed = drone_speed / 3.6
                        travel_time = round(dist / m_per_s_drone_speed, round_number)
                        matrix[current_node - 1][other_node - 1] = travel_time
                        matrix[other_node - 1][current_node - 1] = travel_time
        end_time = time.time()
        processing_time = end_time - start_time
        vprint("processing_time = ", processing_time)
        return matrix

    def plot_graph(self):
        """plot the graph"""
        vprint("==================== PLOT GRAPH ====================")
        # Draw graph
        coordinates = nx.get_node_attributes(self.graph, "coordinates")
        node_colors = []
        count = 0
        for node in self.graph.nodes():
            if self.graph.nodes[node]["demand"] > 0:
                node_colors.append("r")
                count += 1
            else:
                node_colors.append("b")
        nx.draw(self.graph, coordinates, node_color=node_colors, with_labels=True)
        # Show plot
        vprint("number_of_demand_nodes = ", count)
        plt.show()

    def save_map_html(self):
        """plot the nodes on a interactive html map"""
        list_coords = []
        # Create map
        m = folium.Map(location=[44.838633, 0.540983], zoom_start=13)
        # Add points to the map according to the demand
        for _, row in self.nodes.iterrows():
            coord = (row["lat"], row["lon"])
            list_coords.append(coord)
            if row["demand"] > 0:
                folium.Marker(coord, icon=folium.Icon(color="red")).add_to(m)
            else:
                folium.Marker(coord, icon=folium.Icon(color="blue")).add_to(m)
        # Show map
        m.save("assets/map.html")
