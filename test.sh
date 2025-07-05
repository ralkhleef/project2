#!/bin/bash

RUN_MEMCHECK=0

TIMEOUT_COMMAND="timeout 900"
VALGRIND_TIMEOUT_COMMAND="timeout 1800"

IGNORE_TIMEOUT=0

help() {
    echo "Usage: test.sh [options]"
    echo
    echo "Cleans the Cmake cache. If the the option is provided, also completely"
    echo "removes the build director and any build artifacts"
    echo
    echo "Options:"
    echo "  -memcheck          Runs the test with valgrind's memcheck to check for"
    echo "                     memory leaks and other potential memory errors"
    echo "  -ignore-timeout    Ignores the preset timeout length. Intended to"
    echo "                     allow longer running tests to finish."
    echo
    echo "                     WARNING: The extended time IS NOT reflected for"
    echo "                              grading. ONLY use this for custom tests."
    echo "  -help [-h]         Displays this message"
}

new_args=()
for arg in "$@"; do
    if [ "$arg" == "-memcheck" ] || [ "$arg" == "-m" ]; then
        RUN_MEMCHECK=1
    elif [ "$arg" == "-ignore-timeout" ]; then
        echo "========================================================"
        echo "WARNING: The extended time IS NOT reflected for grading."
        echo "         ONLY use this for custom tests."
        echo "========================================================"
        TIMEOUT_COMMAND=""
        VALGRIND_TIMEOUT_COMMAND=""
        IGNORE_TIMEOUT=1
    elif [ "$arg" == "-help" ] || [ "$arg" == "-h" ]; then
        help
        exit 0
    else
        new_args+=("$arg")
    fi
done

SCRIPT_DIR=$(readlink -fn "$(dirname "$0")")

if [ -e "$SCRIPT_DIR/build/46ProjectTests" ]; then
    if [ $RUN_MEMCHECK -eq 1 ]; then
        $TIMEOUT_COMMAND valgrind --tool=memcheck --leak-check=yes --show-reachable=yes --track-origins=yes --suppressions=memcheck.supp "$SCRIPT_DIR/build/46ProjectTests" "${new_args[@]}"
    else
        $VALGRIND_TIMEOUT_COMMAND "$SCRIPT_DIR/build/46ProjectTests" "${new_args[@]}"
    fi
else
    echo "Could not find $SCRIPT_DIR/build/46ProjectTests; have you successfully built?"
    exit 1
fi

if [ $IGNORE_TIMEOUT == 1 ]; then
    echo "========================================================"
    echo "WARNING: The extended time IS NOT reflected for grading."
    echo "         ONLY use this for custom tests."
    echo "========================================================"
fi

