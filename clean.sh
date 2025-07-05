#!/bin/bash

SCRIPT_DIR=$(readlink -fn "$(dirname "$0")")

FULL_CLEAN=0

help() {
    echo "Usage: clean.sh [options]"
    echo
    echo "Cleans the Cmake cache. If the the option is provided, also completely"
    echo "removes the build director and any build artifacts"
    echo
    echo "Options:"
    echo "  -deep [-d]      Performs a deep clean, removing the build directory"
    echo "                  and any build artifacts"
    echo "  -help [-h]      Displays this message"
}

for arg in "$@"; do
    if [ "$arg" == "-deep" ] || [ "$arg" == "-d" ]; then
        FULL_CLEAN=1
    elif [ "$arg" == "-help" ] || [ "$arg" == "-h" ]; then
        help
        exit 0
    else
        echo "Unexpected command line argument. Expected one of: -deep [-d], -help [-h]"
    fi
done

if [ $FULL_CLEAN == 1 ]; then
    rm -rf "$SCRIPT_DIR/build"
    rm -f "$SCRIPT_DIR/compile_commands.json"
    rm -rf "$SCRIPT_DIR/lib/catch2/build"
    rm -rf "$SCRIPT_DIR/.cache/"
else
    cmake --build build --target clean
fi
