cmake_minimum_required(VERSION 3.22.1)
project(medra_bcap)

set(CMAKE_CXX_STANDARD 20)

message ("CMake Compiler: " ${CMAKE_CXX_COMPILER})
message ("CMake Compiler ID: " ${CMAKE_CXX_COMPILER_ID})
message ("CMake Compiler version: " ${CMAKE_CXX_COMPILER_VERSION})

# Compiler flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  message(FATAL_ERROR "MSVC compiler is not supported.")
endif()

message("Using GNU or Clang compiler")
add_compile_options(-Wswitch -Wswitch-enum)  # Checks that all switch cases are covered
add_compile_options(-Wall)  # Enable all warnings
add_compile_options(-Wextra)  # Enable extra warnings

# Set the names and sources of the module
set(MODULE_NAME medra_bcap)
set(PYBIND11_SOURCES 
    src/bindings.cpp
    src/b-Cap.c
    src/DensoController.cpp
    tests/test_execute_servo_trajectory.cpp
)

# Cmake version policy
cmake_policy(SET CMP0057 NEW)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Include dir
include_directories(include)

# Include stdint.h
if(MSVC AND "${_VS_VER}" MATCHES [[v([0-9]?[0-9])]])
  if("${CMAKE_MATCH_1}" LESS 10)
    include_directories(
      include/stdint
    )
  endif("${CMAKE_MATCH_1}" LESS 10)
endif(MSVC AND "${_VS_VER}" MATCHES [[v([0-9]?[0-9])]])


# Include spdlog
include(FetchContent)

FetchContent_Declare(spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(spdlog)


# Pybind
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# Add the pybind module
pybind11_add_module(_${MODULE_NAME} ${PYBIND11_SOURCES})
target_link_libraries(_${MODULE_NAME} PRIVATE spdlog::spdlog)

add_executable(
  test_arm_mutex 
  tests/test_arm_mutex.cpp 
  src/DensoController.cpp 
  src/b-Cap.c
)
target_link_libraries(test_arm_mutex PRIVATE spdlog::spdlog)

add_custom_target(generate_stub ALL
    COMMAND ${Python_EXECUTABLE} "-m" "pybind11_stubgen" "-o${CMAKE_CURRENT_BINARY_DIR}" "_${MODULE_NAME}"
    WORKING_DIRECTORY $<TARGET_FILE_DIR:_${MODULE_NAME}>
    DEPENDS _${MODULE_NAME}
)

# rename the generated .pyi file to __init__.pyi
add_custom_target(rename_pyi ALL
    COMMAND mv ${CMAKE_CURRENT_BINARY_DIR}/_${MODULE_NAME}.pyi ${CMAKE_CURRENT_BINARY_DIR}/__init__.pyi
    WORKING_DIRECTORY $<TARGET_FILE_DIR:_${MODULE_NAME}>
    DEPENDS generate_stub
)

# Install the module
install(TARGETS _${MODULE_NAME} DESTINATION "${MODULE_NAME}")
install(DIRECTORY
    $<TARGET_FILE_DIR:_${MODULE_NAME}>/
    DESTINATION ${MODULE_NAME}
    FILES_MATCHING REGEX "\.pyi$"
)
