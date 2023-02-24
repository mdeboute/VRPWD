#!/bin/bash

valid_methods=(
    [0]="mip heuristic h"
    [1]="mip heuristic h"
    [2]="heuristic h pathheuristic ph"
    [3]="heuristic h"
)

usage() {
    echo "Usage: $(basename "$0") -d <data_dir> -c <case> -m <method>"
    echo "Where <case> is one of the following: 0, 1, 2, 3"
    echo "Where <method> can be:"
    for c in ${!valid_methods[@]}; do
        echo "Case $c:"
        for m in ${valid_methods[$c]}; do
            echo -e "\t$m"
        done
    done
    echo "Example: ./$(basename "$0") -d data/ -c 0 -m mip"
    exit 1
}

parse_arguments() {
    while getopts ":d:c:m:" opt; do
        case ${opt} in
            d )
                data_dir=$OPTARG
            ;;
            c )
                case_num=$OPTARG
            ;;
            m )
                method=$OPTARG
            ;;
            \? )
                usage
            ;;
            : )
                echo "Invalid option: $OPTARG requires an argument" 1>&2
                usage
            ;;
        esac
    done
    shift $((OPTIND -1))
    if [[ -z $data_dir ]] || [[ -z $case_num ]] || [[ -z $method ]]; then
        usage
    fi
}

check_arguments() {
    if [[ ! -d $data_dir ]]; then
        echo "Error: $data_dir is not a directory"
        usage
    fi
    if [[ ! ${valid_methods[$case_num]} =~ (^|[[:space:]])$method($|[[:space:]]) ]]; then
        echo "Error: $method is not a valid method for case $case_num"
        usage
    fi
}

create_campaign() {
    echo "Experimental Campaign:"
    echo "Data directory: $data_dir"
    echo "Output directory: log/"
    echo "Case: $case_num"
    echo "Method: $method"
    mkdir -p log/$method/$case_num
}

run_solver() {
    for instance in "$data_dir"/*; do
        echo "Solving $instance"
        python3 src/vrpwdSolver.py "$instance" "$case_num" "$method" -v > "log/$method/$case_num/result_$(basename "$instance" .txt).log"
    done
}

main() {
    parse_arguments "$@"
    check_arguments
    create_campaign
    run_solver
    echo "Done!"
}

main "$@"
