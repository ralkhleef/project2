#!/bin/bash

SCRIPT_DIR=$(readlink -fn "$(dirname "$0")")

DEBUGGER=lldb

help() {
    echo "Usage: debug.sh [options]"
    echo
    echo "Runs the tests with the lldb debugger"
    echo
    echo "Options:"
    echo "  -debugger=<value> [-d=<value>]   Sets the debugger to use to value. (default lldb)"
    echo "  -help [-h]                       Displays this message"
}


for arg in "$@"; do
    if [ "$arg" == "-help" ] || [ "$arg" == "-h" ]; then
        help
        exit 0
    elif [[ $arg == -debugger=* ]] || [[ $arg == -d=* ]]; then
        DEBUGGER="${arg#=*}"
    fi
done

[ -e "$SCRIPT_DIR/build/46ProjectTests" ] \
    && "$DEBUGGER" "$SCRIPT_DIR/build/46ProjectTests" \
    || { echo "Could not find $SCRIPT_DIR/build/46ProjectTests; have you successfully built?"; exit 1;  }
