import sys
from Data import Data


def print_usage():
    print("Usage: python3 main.py <instance_dir>")


def main():
    if len(sys.argv) != 2 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_usage()
        sys.exit(1)

    instance_dir = sys.argv[1]

    data = Data(instance_dir)
    data.plot_graph()


if __name__ == "__main__":
    main()
