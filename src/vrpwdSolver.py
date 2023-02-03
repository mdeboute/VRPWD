import sys
from VRPWDData import VRPWDData
from TSPGreedy import TSPGreedy
from TSPMIPModel import TSPMIPModel
from VRPWDMIPModel_1 import VRPWDMIPModel_1


def print_usage():
    print("Usage: python3 vrpwdSolver.py <instance_directory> <case> <method>")
    print("Where:")
    print("<instance_directory> is the directory containing the instance files")
    print("<case> is the case of the problem to solve, can be a number in (0, 1, 2, 3)")
    print("<method> is the algorithm to use to solve the case, can be: greedy or mip")
    print("-v or --verbose is an optional argument to print all the information")
    print("-g or --graphic is an optional argument to plot the solution graph")
    print("Example: python3 vrpwdSolver.py data/instance_1/ 0 greedy")


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
    if method != "greedy" and method != "mip":
        print("Method should be greedy or mip")
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
        if method == "greedy":
            solution = TSPGreedy(data).solve()
            if solution.check():
                solution.write()
                if plot:
                    solution.plot()
                    solution.plot_html()
        if method == "mip":
            solution = TSPMIPModel(data).solve()
            if solution.check():
                solution.write()
                if plot:
                    solution.plot()
                    solution.plot_html()
    elif case == 1:
        if method == "mip":
            solution = VRPWDMIPModel_1(data).solve()
            # fixing subtours elimination before building a complete solution and checking it
    else:
        print("Case not implemented yet!")
        sys.exit(1)


if __name__ == "__main__":
    main()
