# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Default ROBOT_ID = 1 if not provided
if(NOT DEFINED ROBOT_ID)
    set(ROBOT_ID 1)
endif()

# Pass ROBOT_ID to C code as preprocessor define
add_compile_definitions(ROBOT_ID=${ROBOT_ID})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robot${ROBOT_ID})
