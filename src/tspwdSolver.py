import sys
from TSPWDData import TSPWDData
from TSPGreedy import TSPGreedy
from TSPMIPModel import TSPMIPModel


def print_usage():
    print("Usage: python3 tspwdSolver.py <instance_directory>")
    print("Example: python3 tspwdSolver.py data/instance_1/")


def main():
    if len(sys.argv) != 2 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_usage()
        sys.exit(1)

    instance_dir = sys.argv[1]
    data = TSPWDData(instance_dir)
    sol_TSPGreedy = TSPGreedy(data).solve()
    sol_TSPGreedy.print_tour()

    sol_TSPMIPModel = TSPMIPModel(data).solve()
    sol_TSPMIPModel.print_tour()


if __name__ == "__main__":
    main()
