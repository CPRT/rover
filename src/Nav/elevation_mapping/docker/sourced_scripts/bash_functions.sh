#!/usr/bin/env bash

print_docker_build()
{
    content=$1
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    echo "$content"
    echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
}

print_docker_run()
{
    content=$1
    echo "==============================================================================="
    echo "$content"
    echo "==============================================================================="
}


function print_color {
    tput setaf $1
    echo "$2"
    tput sgr0
}

function print_error {
    print_color 1 "$1"
}

function print_warning {
    print_color 3 "$1"
}

function print_info {
    print_color 2 "$1"
}

function is_string_in_list {
    string=$1
    list=$2
    for item in ${list//,/ } # Iterate comma seperated list
    do
        if [ "$item" == "$string" ]; then
            echo 1
            return 1
        fi
    done
    echo 0
    return 0
}