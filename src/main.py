import sys
from Data import Data


def print_usage():
    print("Usage: python3 main.py <file_path>")


def main():
    if len(sys.argv) != 2 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_usage()
        sys.exit(1)

    file = sys.argv[1]

    data = Data(file)
    data.plot_graph()


if __name__ == "__main__":
    main()
