import pandas as pd
import geopandas as gpd
from Node import Node
from Edge import Edge
import time
import networkx as nx
import folium
from utils import project_coordinates


class Data(object):
    """This class is used to create the graph from the json file"""

    def __init__(self, file):
        self._brut_df = pd.read_json(file)
        self.nodes = self._create_nodes()
        self.edges = self._create_edges()
        self.graph = self._create_graph()
        self.projected_graph = self._create_projected_graph(4326, 3857)

    def _create_nodes(self):
        start_time = time.time()
        print("==================== NODES CREATION ===================")
        list_nodes = []
        for _, row in self._brut_df.iterrows():
            node_1 = Node(row["lat_min"], row["lon_min"])
            node_2 = Node(row["lat_max"], row["lon_max"])
            list_nodes.append(node_1)
            list_nodes.append(node_2)
        rows_gdf_nodes = []
        cols_gdf_nodes = ["x", "y", "Node Object"]
        for node in list_nodes:
            x = node.lat
            y = node.lon
            rows_gdf_nodes.append({"x": x, "y": y, "Node Object": node})
        gdf_nodes = gpd.GeoDataFrame(rows_gdf_nodes, columns=cols_gdf_nodes)
        # there is duplicate node if brut_gdf_nodes, so we drop them
        gdf_nodes = gdf_nodes.drop_duplicates(subset=["x", "y"], keep="first")
        # reindex the gdf
        gdf_nodes.index = range(1, len(gdf_nodes) + 1)
        end_time = time.time()
        processing_time = end_time - start_time
        print("processing_time: ", processing_time)
        return gdf_nodes

    def _create_edges(self):
        start_time = time.time()
        print("==================== EDGES CREATION ===================")
        # create the gdf edges
        cols_index = ["src", "dest", "key"]
        rows_index = []
        rows_gdf_edges = []
        cols_gdf_edges = ["Edge Object", "speed", "highway"]
        # loop over edges
        for index, row in self._brut_df.iterrows():
            # get the two nodes coordinates
            x = row["lat_min"]
            y = row["lon_min"]
            # search the corresponding node
            row_node = self.nodes.loc[(self.nodes["x"] == x) & (self.nodes["y"] == y)]
            # affecting the node
            src = row_node["Node Object"].iloc[0]

            # same for the other node
            x = row["lat_max"]
            y = row["lon_max"]
            row_node = self.nodes.loc[(self.nodes["x"] == x) & (self.nodes["y"] == y)]
            dest = row_node["Node Object"].iloc[0]
            # add elements in the rows index
            rows_index.append({"src": src, "dest": dest, "key": index})
            # get all attributes for Edge instance
            command_number = row["command_number"]
            internal_id = row["id"]
            source = src
            destination = dest
            length = row["length"]
            oneway = row["oneway"]
            osm_id = row["osmid"]
            osm_type = row["type"]
            if osm_type == "primary":
                speed = 60
            if osm_type == "secondary":
                speed = 45
            else:
                speed = 30
            # create the Edge object
            edge = Edge(
                command_number,
                internal_id,
                source,
                destination,
                length,
                speed,
                oneway,
                osm_id,
                osm_type,
            )
            # add to rows_gdf_edges
            rows_gdf_edges.append(
                {"Edge Object": edge, "speed": speed, "highway": osm_type}
            )
        gdf_edges = gpd.GeoDataFrame(rows_index, columns=cols_index)
        multi_index = pd.MultiIndex.from_frame(gdf_edges)
        gdf_edges = gpd.GeoDataFrame(
            rows_gdf_edges, index=multi_index, columns=cols_gdf_edges
        )
        end_time = time.time()
        processing_time = end_time - start_time
        print("processing_time: ", processing_time)
        return gdf_edges

    def _create_graph(self):
        print("================= GRAPH CREATION ====================")
        # create empty directed graph
        graph = nx.DiGraph()
        # add nodes
        for _, row in self.nodes.iterrows():
            node = row["Node Object"]
            coord = (node.lat, node.lon)
            graph.add_node(coord)
        # add edges
        for _, row in self.edges.iterrows():
            edge = row["Edge Object"]
            graph.add_edge(edge.source, edge.destination)
        # print graph info
        print("graph: ", graph)
        return graph

    def _create_projected_graph(self, initial_referential, output_referential):
        """create the graph from nodes and edges
        input: None
        output: Directed Graph Object from Networkx
        method: convert GPS coordinates into cartesian coordinates and then create a directed graph"""
        print("================ PROJECTED GRAPH CREATION =================")
        # create lists to store coordinates
        list_coords_gps = []
        list_coords_projected = []
        for index, row in self.nodes.iterrows():
            node = row["Node Object"]
            coord = (node.lat, node.lon)
            list_coords_gps.append(coord)

        # convert GPS coordinates to projected coordinates
        list_coords_projected = project_coordinates(
            list_coords_gps, initial_referential, output_referential
        )
        print("list_coords_projected: ", list_coords_projected)
        # create empty directed graph
        graph = nx.DiGraph()
        # add nodes
        for index, row in self.nodes.iterrows():
            node_index = index
            x = list_coords_projected[index - 1][0]
            y = list_coords_projected[index - 1][1]
            graph.add_node(node_index, coordinates=(x, y))
        # add edges
        for idx, row in self.edges.iterrows():
            edge = row["Edge Object"]
            source = edge.source
            index_source = self.nodes.query(
                "x == {} and y == {}".format(source.lat, source.lon)
            ).index[0]
            destination = edge.destination
            index_destination = self.nodes.query(
                "x == {} and y == {}".format(destination.lat, destination.lon)
            ).index[0]
            graph.add_edge(index_source, index_destination, index=idx)
        # print graph info
        print("graph: ", graph)
        return graph

    def plot_graph(self):
        """plot the graph"""
        list_coords = []
        for _, row in self.nodes.iterrows():
            node = row["Node Object"]
            coord = (node.lat, node.lon)
            list_coords.append(coord)
        # Create map
        m = folium.Map(location=[44.838633, 0.540983], zoom_start=13)
        # Add points to map
        for coords in list_coords:
            folium.Marker(coords).add_to(m)
        # Show map
        m.save("assets/map.html")
