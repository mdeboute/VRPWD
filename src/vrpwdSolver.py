import sys

from VRPWDData import VRPWDData
from TSPGreedy import TSPGreedy
from TSPMIPModel import TSPMIPModel
from VRPWDMIPModel_1 import VRPWDMIPModel_1
from VRPWDHeuristic_1 import VRPWDHeuristic_1
from VRPWDHeuristic_2 import VRPWDHeuristic_2
from VRPWDPathHeuristic_2 import VRPWDPathHeuristic_2


def print_usage():
    print("Usage: python3 vrpwdSolver.py <instance_directory> <case> <method>")
    print("Where:")
    print("<instance_directory> is the directory containing the instance files")
    print("<case> is the case of the problem to solve, can be a number in (0, 1, 2, 3)")
    print(
        "<method> is the algorithm to use to solve the case, can be: heuristic or mip"
    )
    print("-v or --verbose is an optional argument to print all the information")
    print("-g or --graphic is an optional argument to plot the solution graph")
    print("Example: python3 vrpwdSolver.py data/instance_1/ 0 heuristic")


def main():
    if len(sys.argv) < 4 or ("-h" in sys.argv) or ("--help" in sys.argv):
        print_usage()
        sys.exit(1)

    instance_dir = sys.argv[1]

    case = int(sys.argv[2])
    if case < 0 or case > 3:
        print("Case should be a number in (0, 1, 2, 3)")
        print("Please use -h or --help to see the usage")
        sys.exit(1)

    method = sys.argv[3]
    if method != "heuristic" and method != "mip" and method != "pathheuristic" :
        print("Method should be heuristic or mip")
        print("Please use -h or --help to see the usage")
        sys.exit(1)

    if "-v" in sys.argv or "--verbose" in sys.argv:
        verbose = True
    else:
        verbose = False

    if "-g" in sys.argv or "--graphic" in sys.argv:
        plot = True
    else:
        plot = False

    data = VRPWDData(instance_dir, case, verbose)
    data.save_map_html()

    if case == 0:
        if method == "heuristic":
            solution = TSPGreedy(data).solve()
            if solution.check():
                solution.write()
                if plot:
                    solution.plot()

        if method == "mip":
            solution = TSPMIPModel(data).solve()
            if solution.check():
                solution.write()
                if plot:
                    solution.plot()

    elif case == 1:
        if method == "mip":
            solution = VRPWDMIPModel_1(data).solve()
            # TODO: fix subtours elimination before building a complete solution and checking it

        if method == "heuristic":
            solution = VRPWDHeuristic_1(data).solve()
    elif case == 2:
        if method == "heuristic":
            solution = VRPWDHeuristic_2(data,2).solve()
        if method == "pathheuristic":
            solution = VRPWDPathHeuristic_2(data).solve()
    else:
        print("Case not implemented yet!")
        sys.exit(1)


if __name__ == "__main__":
    main()
