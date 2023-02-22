import sys

from VRPWDData import VRPWDData
from TSPGreedy import TSPGreedy
from TSPMIPModel import TSPMIPModel
from VRPWDReducedMIPModel_1 import VRPWDReducedMIPModel_1
from VRPWDHeuristic_1 import VRPWDHeuristic_1
from VRPWDHeuristic_2 import VRPWDHeuristic_2
from VRPWDPathHeuristic_2 import VRPWDPathHeuristic_2
from utils import verbose_print


def print_usage():
    print("Usage: python3 vrpwdSolver.py <instance_directory> <case> <method>")
    print("Where:")
    print("<instance_directory> is the directory containing the instance files")
    print("<case> is the case of the problem to solve, can be a number in (0, 1, 2, 3)")
    print(
        "<method> is the algorithm to use to solve the case, can be: heuristic or mip (h for short)"
    )
    print(
        "-v or --verbose is an optional argument to print all the information of the execution"
    )
    print("-g or --graphic is an optional argument to plot the solution graph")
    print("Example: python3 vrpwdSolver.py data/instance_1/ 0 heuristic")


def main():
    if (
        len(sys.argv) < 4
        or len(sys.argv) > 6
        or ("-h" in sys.argv)
        or ("--help" in sys.argv)
    ):
        print_usage()
        sys.exit(1)

    instance_dir = sys.argv[1]
    case = int(sys.argv[2])

    if case < 0 or case > 3:
        print("Case should be a number in (0, 1, 2, 3)!")
        print("Please use -h or --help to see the usage")
        sys.exit(1)

    method = sys.argv[3]

    if (
        method != "heuristic"
        and method != "mip"
        and method != "h"
        and method != "pathheuristic"
    ):
        print("Method should be heuristic or mip!")
        print("Please use -h or --help to see the usage")
        sys.exit(1)

    verbose = False

    if "-v" in sys.argv or "--verbose" in sys.argv:
        verbose = True

    global vprint
    vprint = verbose_print(verbose)

    plot = False

    if "-g" in sys.argv or "--graphic" in sys.argv:
        plot = True

    data = VRPWDData(instance_dir, case, verbose)
    data.save_map_html()

    if case == 0:
        if method == "heuristic" or method == "h":
            solution = TSPGreedy(data).solve()
            if solution.check():
                print(
                    f"Result: runtime={solution.runtime:.2f}sec; objective={solution.objective_value:.2f}sec"
                )
                solution.write()
                if plot:
                    solution.plot()

        if method == "mip":
            solution = TSPMIPModel(data).solve()
            if solution.check():
                print(
                    f"Result: runtime={solution.runtime:.2f}sec; objective={solution.objective_value:.2f}sec; gap={solution.gap:.4f}%"
                )
                solution.write()
                if plot:
                    solution.plot()

    elif case == 1:
        if method == "mip":
            solution = VRPWDReducedMIPModel_1(data).solve()
            # TODO: fix subtours elimination before building a complete solution and checking it
            if solution.check():
                print(
                    f"Result: runtime={solution.runtime:.2f}sec; objective={solution.objective_value:.2f}sec; gap={solution.gap:.4f}%"
                )
                solution.write()
                if plot:
                    solution.plot()

        if method == "heuristic" or method == "h":
            solution = VRPWDHeuristic_1(data).solve()
            if solution.check():
                print(
                    f"Result: runtime={solution.runtime:.2f}sec; objective={solution.objective_value:.2f}sec"
                )
                solution.write()
                if plot:
                    solution.plot()
    elif case == 2:
        if method == "heuristic" or method == "h":
            solution = VRPWDHeuristic_2(data, 2).solve()
            # if solution.check():
            #     print(
            #         f"Result: runtime={solution.runtime:.2f}sec; objective={solution.objective_value:.2f}sec"
            #     )
            #     solution.write()
            if plot:
                solution.plot()
        if method == "pathheuristic":
            solution = VRPWDPathHeuristic_2(data).solve()
            if solution.check():
                print(
                    f"Result: runtime={solution.runtime:.2f}sec; objective={solution.objective_value:.2f}sec"
                )
                solution.write()
                if plot:
                    solution.plot()
    else:
        print("Case not implemented yet! Check the usage with -h or --help.")
        sys.exit(1)


if __name__ == "__main__":
    main()
