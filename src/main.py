import sys
from Data import Data


def print_usage():
    print("Usage: python3 main.py <instance_directory>")
    print("Example: python3 main.py data/instance_1/")


def main():
    if len(sys.argv) != 2 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_usage()
        sys.exit(1)

    instance_dir = sys.argv[1]
    data = Data(instance_dir)


if __name__ == "__main__":
    main()
