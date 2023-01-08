import pandas as pd
import geopandas as gpd
from Node import Node
from Edge import Edge
import time
import networkx as nx
import matplotlib.pyplot as plt
import folium
from utils import project_coordinates

# -------------------------------------------------------
class Data(object):
    """Class which contains data for the problem from an instance file"""

    # ---------------------------------------------------
    def __init__(self, file):
        self.brut_df = pd.read_json(file)
        self.nodes = (
            self._create_nodes()
        )  # current : gdf of all nodes without duplicates, better list of Nodes : [Node, Node,...] ???
        self.edges = self._create_edges()
        self.graph = self._create_graph()
        self.projected_graph = self._create_projected_graph(4326, 3857)

    # ---------------------------------------------------
    def _create_nodes(self):
        """create all nodes from the instance file"""
        start_time = time.time()
        print("==================== NODES CREATION ===================")
        # create the list of nodes
        list_nodes = []
        round_number = 7
        for _, row in self.brut_df.iterrows():
            lon_min = round(row["lon_min"], round_number)
            lat_min = round(row["lat_min"], round_number)
            lon_max = round(row["lon_max"], round_number)
            lat_max = round(row["lat_max"], round_number)
            node1 = Node(lat_min, lon_min)
            node2 = Node(lat_max, lon_max)
            list_nodes.append(node1)
            list_nodes.append(node2)
        # print('list_nodes at the end of row : ',len(list_nodes))
        # create the gdf nodes
        rows_gdf_nodes = []
        cols_gdf_nodes = ["x", "y", "Node Object"]
        for node in list_nodes:
            x = node.get_lon()
            y = node.get_lat()
            rows_gdf_nodes.append({"x": x, "y": y, "Node Object": node})
        brut_gdf_nodes = gpd.GeoDataFrame(rows_gdf_nodes, columns=cols_gdf_nodes)
        # there is duplicate node if brut_gdf_nodes, so we drop them
        gdf_nodes_without_duplicates = brut_gdf_nodes.drop_duplicates(
            subset=["x", "y"], keep="first"
        )
        # reindex the gdf
        gdf_nodes_without_duplicates.index = range(
            1, len(gdf_nodes_without_duplicates) + 1
        )
        end_time = time.time()
        processing_time = end_time - start_time
        print("processing_time: ", processing_time)
        return gdf_nodes_without_duplicates

    def _create_edges(self):
        """create all edges from the instance file"""
        start_time = time.time()
        print("==================== EDGES CREATION ===================")
        # create the gdf edges
        cols_index = ["u", "v", "key"]
        rows_index = []
        rows_gdf_edges = []
        cols_gdf_edges = ["Edge Object", "speed", "highway"]
        i = 1
        round_number = 7
        # loop over edges
        print("len(self.brut_df): ", len(self.brut_df))
        for _, row in self.brut_df.iterrows():
            list_t = []
            # get the two nodes coordinates
            lon_min = round(row["lon_min"], round_number)
            lat_min = round(row["lat_min"], round_number)
            # search the corresponding node
            row_node = self.nodes.loc[
                (self.nodes["x"] == lon_min) & (self.nodes["y"] == lat_min)
            ]
            # print('row = ',row_node)
            # affeting the node
            u = row_node["Node Object"].iloc[0]
            # same for the other node
            lon_max = round(row["lon_max"], round_number)
            # print('lon_max = ',lon_max)
            lat_max = round(row["lat_max"], round_number)
            row_node = self.nodes.loc[
                (self.nodes["x"] == lon_max) & (self.nodes["y"] == lat_max)
            ]
            v = row_node["Node Object"].iloc[0]
            # add elements in the rows index
            rows_index.append({"u": u, "v": v, "key": i})
            i = i + 1
            # get all attributes for Edge instance
            command_number = row["command_number"]
            internal_id = row["id"]
            from_node = u
            to_node = v
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
                from_node,
                to_node,
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
        d = gpd.GeoDataFrame(rows_index, columns=cols_index)
        multi_index = pd.MultiIndex.from_frame(d)
        gdf_edges = gpd.GeoDataFrame(
            rows_gdf_edges, index=multi_index, columns=cols_gdf_edges
        )
        end_time = time.time()
        processing_time = end_time - start_time
        print("processing_time: ", processing_time)
        return gdf_edges

    # ---------------------------------------------------
    def _create_graph(self):
        """create the graph from nodes and edges"""
        print("================= GRAPH CREATION ====================")
        # create empty directed graph
        graph = nx.DiGraph()
        # add nodes
        for _, row in self.nodes.iterrows():
            node = row["Node Object"]
            coords = (node.get_lat(), node.get_lon())
            graph.add_node(coords)
        # add edges
        for _, row in self.edges.iterrows():
            edge = row["Edge Object"]
            source = edge.get_from_node()
            target = edge.get_to_node()
            graph.add_edge(source, target)
        # print graph info
        print("graph: ", graph)
        return graph

    # ---------------------------------------------------
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
            coord = (node.get_lon(), node.get_lat())
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
            source = edge.get_from_node()
            index_source = self.nodes.query(
                "x == {} and y == {}".format(source.get_lon(), source.get_lat())
            ).index[0]
            target = edge.get_to_node()
            index_target = self.nodes.query(
                "x == {} and y == {}".format(target.get_lon(), target.get_lat())
            ).index[0]
            graph.add_edge(index_source, index_target, index=idx)
        # print graph info
        print("graph: ", graph)
        return graph

    # ---------------------------------------------------
    def plot_graph(self):
        """plot the graph"""
        # Draw graph
        nx.draw(self.graph, with_labels=True)
        # Show plot
        plt.show()

    # ---------------------------------------------------
    def plot_graph_v2(self):
        """plot the graph"""
        list_coords = []
        for _, row in self.nodes.iterrows():
            node = row["Node Object"]
            coord = (node.get_lat(), node.get_lon())
            list_coords.append(coord)
        # Create map
        m = folium.Map(location=[44.838633, 0.540983], zoom_start=13)
        # Add points to map
        for coords in list_coords:
            folium.Marker(coords).add_to(m)
        # Show map
        m.save("map.html")
