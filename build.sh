#!/bin/bash

SCRIPT_DIR=$(readlink -fn "$(dirname "$0")")
OUT_DIR=$SCRIPT_DIR/build

COMPILER_CXX=clang++
COMPILER_C=clang
CXX_STANDARD=20
REQUIRE_STANDARD=ON
ENABLE_EXTENSIONS=OFF
GENERATE_COMPILE_COMMANDS=ON

CONFIGURE_ONLY=0

help() {
    echo "Usage: build.sh [options]"
    echo
    echo "Configures and builds the project for the class."
    echo
    echo "Options:"
    echo "  -configure [-c]                     The script will only configure the"
    echo "                                      project if this is set"
    echo "  -cxx-compiler=<value>               Sets the cpp compiler to what is passed"
    echo "                                      in to value (default clang++)"
    echo "  -c-compiler=<value>                 Sets the c compiler to what is pass in"
    echo "                                      to value (default clang)"
    echo "  -compiler=<value>                   If value equals gcc, sets cpp compiler"
    echo "                                      to g++ and c compiler to gcc"
    echo "                                      If value equals clang, sets cpp compiler"
    echo "                                      to clang++ and c compiler to clang (default)"
    echo "  -std=<value>                        Sets the cpp standard to value (default 20)"
    echo "  -require-std=<value>                If value is ON, requires standard compliance (default)"
    echo "                                      If value is OFF, allows non-standard compliance"
    echo "  -enable-extensions=<value>          If value is ON, enables extensions"
    echo "                                      If value is OFF, disables extensions"
    echo "  -generate-compile-commands=<value>  If value is ON, generates compile commands (default)"
    echo "      [-cc=<value>]                   If value is OFF, don't generate compile commands"
    echo "  -help [-h]                          Displays this message"
}

new_args=()
for arg in "$@"; do
    if [ "$arg" == "-configure" ] || [ "$arg" == "-c" ]; then
        CONFIGURE_ONLY=1
    elif [[ $arg = -cxx-compiler=* ]]; then
        COMPILER_CXX="${arg#*=}"
    elif [[ $arg = -c-compiler=* ]]; then
        COMPILER_C="${arg#*=}"
    elif [[ $arg = -compiler=* ]]; then
        compiler="${arg#*=}"
        if [ "$compiler" == "gcc" ]; then
            COMPILER_CXX=g++
            COMPILER_C=gcc
        elif [ "$compiler" == "clang" ]; then
            COMPILER_CXX=clang++
            COMPILER_C=clang
        fi
    elif [[ $arg = -std=* ]]; then
        CXX_STANDARD="${arg#*=}"
    elif [[ $arg = -require-std=* ]]; then
        REQUIRE_STANDARD="${arg#*=}"
    elif [[ $arg = -enable-extensions=* ]]; then
        ENABLE_EXTENSIONS="${arg#*=}"
    elif [[ $arg = -generate-compile-commands=* ]] || [[ $arg = -cc=* ]]; then
        GENERATE_COMPILE_COMMANDS="${arg#*=}"
    elif [ "$arg" == "-h" ] || [ "$arg" == "-help" ]; then
        help
        exit 0
    else
        new_args+=("$arg")
    fi
done

if [ $CONFIGURE_ONLY == 1 ]; then
    mkdir -p "$OUT_DIR"
    cmake -S "$SCRIPT_DIR" -B "$OUT_DIR" \
            -D CMAKE_C_COMPILER="$COMPILER_C" \
            -D CMAKE_CXX_COMPILER="$COMPILER_CXX" \
            -D CMAKE_CXX_STANDARD="$CXX_STANDARD" \
            -D CMAKE_CXX_STANDARD_REQUIRED="$REQUIRE_STANDARD" \
            -D CMAKE_CXX_EXTENSIONS="$ENABLE_EXTENSIONS" \
            -D CMAKE_EXPORT_COMPILE_COMMANDS="$GENERATE_COMPILE_COMMANDS" \
            "${new_args[@]}" \
        && if [[ "$OSTYPE" == "darwin"* ]]; then \
            sed -i '' 's/\(SHINDLER_ICS46_WARNINGS_AS_ERRORS:BOOL=\).*/\1TRUE/' build/CMakeCache.txt; \
           else \
            sed -i 's/\(SHINDLER_ICS46_WARNINGS_AS_ERRORS:BOOL=\).*/\1TRUE/' build/CMakeCache.txt; \
           fi \
        && (test -f "$OUT_DIR/compile_commands.json"  \
        && ln -sf "$OUT_DIR/compile_commands.json" "$SCRIPT_DIR/compile_commands.json" \
            || exit 0)
    exit $?
fi

mkdir -p "$OUT_DIR"
cmake -S "$SCRIPT_DIR" -B "$OUT_DIR" \
        -D CMAKE_C_COMPILER="$COMPILER_C" \
        -D CMAKE_CXX_COMPILER="$COMPILER_CXX" \
        -D CMAKE_CXX_STANDARD="$CXX_STANDARD" \
        -D CMAKE_CXX_STANDARD_REQUIRED="$REQUIRE_STANDARD" \
        -D CMAKE_CXX_EXTENSIONS="$ENABLE_EXTENSIONS" \
        -D CMAKE_EXPORT_COMPILE_COMMANDS="$GENERATE_COMPILE_COMMANDS" \
        "${new_args[@]}" \
    && cmake --build "$OUT_DIR" \
    && if [[ "$OSTYPE" == "darwin"* ]]; then \
        sed -i '' 's/\(SHINDLER_ICS46_WARNINGS_AS_ERRORS:BOOL=\).*/\1TRUE/' build/CMakeCache.txt; \
       else \
        sed -i 's/\(SHINDLER_ICS46_WARNINGS_AS_ERRORS:BOOL=\).*/\1TRUE/' build/CMakeCache.txt; \
       fi \
    && (test -f "$OUT_DIR/compile_commands.json" \
        && ln -sf "$OUT_DIR/compile_commands.json" "$SCRIPT_DIR/compile_commands.json" \
        || exit 0)
