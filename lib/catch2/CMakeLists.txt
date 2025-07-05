cmake_minimum_required(VERSION 3.10)

project(Catch2 VERSION 3.5.1 LANGUAGES CXX)

add_library(Catch2Amalgamated src/catch2/catch_amalgamated.cpp)
target_include_directories(Catch2Amalgamated PUBLIC src/)
target_compile_features(Catch2Amalgamated PUBLIC cxx_std_14)
add_library(Catch2::Amalgamated ALIAS Catch2Amalgamated)

