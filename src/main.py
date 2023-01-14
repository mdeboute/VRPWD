import sys
from Data import Data


def print_usage():
    print("Usage: python3 main.py <map_file_path> <demands_file_path>")


def main():
    if len(sys.argv) != 3 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_usage()
        sys.exit(1)

    map_file = sys.argv[1]
    demands_file = sys.argv[2]

    data = Data(map_file, demands_file)
    data.plot_graph()


if __name__ == "__main__":
    main()
