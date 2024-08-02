cmake_minimum_required(VERSION 3.22.1)
project(denso_pybind)

message ("CMake Compiler: " ${CMAKE_CXX_COMPILER})
message ("CMake Compiler ID: " ${CMAKE_CXX_COMPILER_ID})
message ("CMake Compiler version: " ${CMAKE_CXX_COMPILER_VERSION})

# Set the names and sources of the module
set(MODULE_NAME denso_pybind)
set(SOURCES 
    src/bindings.cpp
    src/b-Cap.c
    src/DensoController.cpp
    tests/test_execute_servo_trajectory.cpp
    )

# Cmake version policy
cmake_policy(SET CMP0057 NEW)

# Include dir
include_directories(
  include
)

# Pybind
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# Specify path to Orin bCAP library and its headers
# set(EXTERNAL_LIB_PATH /workspaces/orin_bcap/C)
# set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/build/libbcap_client.so)
# set(EXTERNAL_INCLUDE_PATH ${EXTERNAL_LIB_PATH}/include)


# Add the pybind module
pybind11_add_module(${MODULE_NAME} ${SOURCES})

# Include the bCAP headers
# target_include_directories(${MODULE_NAME} PRIVATE ${EXTERNAL_INCLUDE_PATH})

add_executable(test_execute_servo_trajectory tests/test_execute_servo_trajectory.cpp src/DensoController.cpp src/b-Cap.c)
target_include_directories(test_execute_servo_trajectory PRIVATE include)
# target_link_libraries(test_execute_servo_trajectory PRIVATE ${EXTERNAL_LIB})

# Link the bCAP library
# target_link_libraries(${MODULE_NAME} PRIVATE ${EXTERNAL_LIB})

# Install the module
install(TARGETS ${MODULE_NAME} DESTINATION .)