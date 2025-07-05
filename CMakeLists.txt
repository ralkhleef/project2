cmake_minimum_required(VERSION 3.22)

option(SHINDLER_ICS46_SET_COMPILE_FLAGS "Whether or not to set the compile flags for the class" ON)
option(SHINDLER_ICS46_WARNINGS_AS_ERRORS "Wherther or not to set the compiler to mark warnings as errors" ON)
set(SHINDLER_ICS46_COMPILE_FLAGS -gdwarf-4 -Wall -pedantic-errors -Wextra -glldb -O0)
if (SHINDLER_ICS46_WARNINGS_AS_ERRORS)
    set(SHINDLER_ICS46_COMPILE_FLAGS ${SHINDLER_ICS46_COMPILE_FLAGS} -Werror)
else()
    message(WARNING "Warnings as errors has been disabled. This does not match the grading computer.")
endif()

project(46Project)

# LIBRARY

file(GLOB_RECURSE APP_SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE JUPYTER_CHECKPOINT_FILES */.ipynb_checkpoints/*)
list(REMOVE_ITEM APP_SRC_FILES ${JUPYTER_CHECKPOINT_FILES})

if (APP_SRC_FILES)
    add_library(${PROJECT_NAME}Library ${APP_SRC_FILES})
    target_include_directories(${PROJECT_NAME}Library PUBLIC ${PROJECT_SOURCE_DIR}/src)
    target_compile_features(${PROJECT_NAME}Library PRIVATE cxx_std_20)
    if(SHINDLER_ICS46_SET_COMPILE_FLAGS)
        target_compile_options(${PROJECT_NAME}Library PRIVATE ${SHINDLER_ICS46_COMPILE_FLAGS})
    endif()
else()
    add_library(${PROJECT_NAME}Library INTERFACE)
    target_include_directories(${PROJECT_NAME}Library INTERFACE ${PROJECT_SOURCE_DIR}/src)
    target_compile_features(${PROJECT_NAME}Library INTERFACE cxx_std_20)
    if(SHINDLER_ICS46_SET_COMPILE_FLAGS)
        target_compile_options(${PROJECT_NAME}Library INTERFACE ${SHINDLER_ICS46_COMPILE_FLAGS})
    endif()
endif()
add_library(${PROJECT_NAME}::lib ALIAS ${PROJECT_NAME}Library)

# TESTS

add_subdirectory(lib/catch2/)

file(GLOB TEST_SRC_FILES ${CMAKE_SOURCE_DIR}/tst/*.cpp)
list(REMOVE_ITEM TEST_SRC_FILES ${JUPYTER_CHECKPOINT_FILES})

add_executable(${PROJECT_NAME}Tests ${TEST_SRC_FILES})
target_compile_features(${PROJECT_NAME}Tests PUBLIC cxx_std_20)
if(SHINDLER_ICS46_SET_COMPILE_FLAGS)
    target_compile_options(${PROJECT_NAME}Tests PRIVATE ${SHINDLER_ICS46_COMPILE_FLAGS})
endif()
target_include_directories(${PROJECT_NAME}Tests PRIVATE ${PROJECT_SOURCE_DIR}/tst)
target_link_libraries(${PROJECT_NAME}Tests PRIVATE ${PROJECT_NAME}Library Catch2::Amalgamated)
add_executable(${PROJECT_NAME}::tst ALIAS ${PROJECT_NAME}Tests)

if (NOT SHINDLER_ICS46_WARNINGS_AS_ERRORS)
    add_custom_target(WarningsAsErrorsCheck ALL 
        ${CMAKE_COMMAND} -E cmake_echo_color --yellow 
        "Warnings as errors has been disabled.  This does not match the grading computer.")
    add_dependencies(WarningsAsErrorsCheck ${PROJECT_NAME}Library ${PROJECT_NAME}Tests)
endif()
