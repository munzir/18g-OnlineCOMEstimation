cmake_minimum_required(VERSION 2.8.12)

project(balancingWithCoMError)

# Stuff needed to get Octave working
#list (APPEND CMAKE_MODULE_PATH "./cmake/FindOCTAVE.cmake")
# set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
#INCLUDE_DIRECTORIES(
#    /usr/include/octave-4.0.0/
#)
#LINK_DIRECTORIES(
#    /usr/include/octave-4.0.0/
#)

find_package(DART 6 REQUIRED COMPONENTS utils-urdf gui CONFIG)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS} ${OCTAVE_OCT_LIB_DIR})

# file(GLOB srcs "Main.cpp" "Main.hpp" "MyWindow.cpp" "MyWindow.hpp" "Controller.cpp" "Controller.hpp" "gen_phi_matrix.cpp" "gen_phi_matrix.hpp" "file_ops.cpp" "file_ops.hpp" "lqr.cpp" "lqr.hpp")
file(GLOB srcs "Main.cpp" "Main.hpp" "MyWindow.cpp" "MyWindow.hpp" "Controller.cpp" "Controller.hpp" "gen_phi_matrix.cpp" "gen_phi_matrix.hpp" "file_ops.cpp" "file_ops.hpp")
add_executable(${PROJECT_NAME} ${srcs})

# target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt config4cpp octave octinterp)
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt config4cpp)
