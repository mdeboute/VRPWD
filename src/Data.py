import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
from geopy.distance import geodesic
from Node import Node
from Edge import Edge
import time
import networkx as nx
import folium
from utils import project_coordinates


class Data(object):
    """This class is used to create the graph from the json file"""
    def __init__(self, map_file, demands_file):
        self._brut_df_map = pd.read_json(map_file)
        self._brut_df_demand = pd.read_json(demands_file)
        self.nodes = self._create_nodes_and_edges_df()[0]
        self.edges = self._create_nodes_and_edges_df()[1]
        self.graph = self._create_graph()
        #self.projected_graph = self._create_projected_graph(4326, 3857)

    def _create_nodes_and_edges_df(self):
        start_time = time.time()
        print("==================== NODES AND EDGES CREATION ===================")
        list_coords = []
        rows_gdf_nodes = []
        cols_gdf_nodes = ["lat", "lon", "demand"]
        rows_gdf_edges = []
        cols_gdf_edges = ["src", "dest", "length", "speed", "osm_id", "osm_type","travel_time"]
        for _, row in self._brut_df_map.iterrows():
            if (row["lat_min"],row["lon_min"]) in list_coords:
                idx_src=list_coords.index((row["lat_min"],row["lon_min"]))+1
            else:
                list_coords.append((row["lat_min"],row["lon_min"]))
                idx_src=len(list_coords)
            if (row["lat_max"],row["lon_max"]) in list_coords:
                idx_dest=list_coords.index((row["lat_max"],row["lon_max"]))+1
            else:
                list_coords.append((row["lat_max"],row["lon_max"]))
                idx_dest=len(list_coords)
            length=row["length"]
            osm_id=row["osmid"]
            osm_type=row["type"]
            if osm_type == "primary":
                speed = 60
            if osm_type == "secondary":
                speed = 45
            else:
                speed = 30
            m_per_s_speed=round(speed/3.6,2)
            travel_time=length/m_per_s_speed
            rows_gdf_edges.append({
                "src":idx_src,
                "dest":idx_dest,
                "length":length,
                "speed":speed,
                "osm_id":osm_id,
                "osm_type":osm_type,
                "travel_time":travel_time
            }) 
        gdf_edges=gpd.GeoDataFrame(rows_gdf_edges,columns=cols_gdf_edges)
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
        print("processing_time: ", processing_time)
        print(gdf_nodes)
        print(gdf_edges)
        return gdf_nodes,gdf_edges

    def _create_graph(self):
        print("================= GRAPH CREATION ====================")
        start_time=time.time()
        # create empty directed graph
        graph = nx.DiGraph()
        # add nodes
        for idx, row in self.nodes.iterrows():
            lat = row["lat"]
            lon = row["lon"]
            coord = (lon,lat)
            d=row["demand"]
            graph.add_node(idx,coordinates=coord,demand=d)
        # add edges
        for idx, row in self.edges.iterrows():
            src=row["src"]
            dest=row["dest"]
            travel_time=row["travel_time"]
            graph.add_edge(src, dest, id=idx,travel_time=travel_time)
        # print graph info
        print("graph: ", graph)
        #add demand
        for _,row in self._brut_df_demand.iterrows():
            point=(row["lat"],row["lon"])
            demand=row['amount']
            nearest_node=None
            nearest_distance=float('inf')
            for node in graph.nodes:
                node_coord=graph.nodes[node]['coordinates']
                node_coords_inversed=(node_coord[1],node_coord[0])
                #compute euclidian distance through Haversine formula
                dist=geodesic(point,node_coords_inversed).km
                if dist<nearest_distance:
                    nearest_distance=dist
                    nearest_node=node
            #update the demand
            #in the graph
            graph.nodes[nearest_node].update({'demand': demand})
            #in the df
            self.nodes.loc[nearest_node,'demand']=demand
        end_time=time.time()
        processing_time=end_time-start_time
        print('processing_time = ',processing_time)
        return graph
    
    def plot_graph(self):
        """plot the graph"""
        print('#=============PLOT GRAPH====================')
        # Draw graph
        coordinates = nx.get_node_attributes(self.graph, 'coordinates')
        node_colors = []
        count=0
        for node in self.graph.nodes():
            #print(self.graph.nodes[node])
            if self.graph.nodes[node]['demand'] > 0:
                node_colors.append('r')
                count+=1
            else:
                node_colors.append('b')
        nx.draw(self.graph,coordinates,node_color=node_colors, with_labels=True)
        # Show plot
        print('nb demand node = ',count)
        plt.show()

    def plot_nodes_html(self):
        """plot the nodes on a interactive html map"""
        list_coords = []
        # Create map
        m = folium.Map(location=[44.838633, 0.540983], zoom_start=13)
        for _, row in self.nodes.iterrows():
            coord = (row['lat'], row['lon'])
            list_coords.append(coord)
            if row['demand']>0: 
                folium.Marker(coord,icon=folium.Icon(color='red')).add_to(m)
            else:
                folium.Marker(coord,icon=folium.Icon(color='blue')).add_to(m)
        # Add points to map
        for coords in list_coords:
            folium.Marker(coords).add_to(m)
        # Show map
        m.save("assets/map.html")
