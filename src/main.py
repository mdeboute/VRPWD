import sys
from Data import Data
from utils import project_coordinates


def print_usage():
    print("Usage: python3 main.py <file_path>")


def main():
    if len(sys.argv) != 2 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_usage()
        sys.exit(1)

    file = sys.argv[1]

    data = Data(file)
    print(data.graph)
    data.plot_graph()
    # initial_coords = []
    # for node in data.nodes:
    #     coord_init = node[1]["coordinates"]
    #     initial_coords.append(coord_init)
    # list_coords_gps_from_cart_proj = project_coordinates(initial_coords, 3857, 4326)
    # coords = []
    # for _, row in data.get_nodes().iterrows():
    #     lon = row["Node Object"].get_lon()
    #     lat = row["Node Object"].get_lat()
    #     c = (lon, lat)
    #     coords.append(c)
    # if list_coords_gps_from_cart_proj == coords:
    #     print("reverse projection succeed!")
    # else:
    #     print("doesnt succeed...")  # search where are the differences
    # print("len(list_coords_gps_from_cart_proj): ", len(list_coords_gps_from_cart_proj))
    # print("len(coords): ", len(coords))
    # # print('list_coords_gps_from_cart_proj = ',list_coords_gps_from_cart_proj)
    # for t1, t2 in zip(coords, list_coords_gps_from_cart_proj):
    #     print("initial: {} / coordinates from proj: {} ".format(t1, t2))


if __name__ == "__main__":
    main()
