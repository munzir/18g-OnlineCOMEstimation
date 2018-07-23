cmake_minimum_required(VERSION 2.8.12)

project(lqrTest)

# Stuff needed to get Octave working

find_program(MKOCTFILE_EXECUTABLE mkoctfile)
if(NOT MKOCTFILE_EXECUTABLE)
    message(SEND_ERROR "Failed to find mkoctfile, is it in your $PATH?")
endif()

#set(MKOCTFILE "mkoctfile")
#set(OCTARG "--link-stand-alone")
#add_custom_command(TARGET reordering-library
#       PRE_LINK
#       COMMAND ${MKOCTFILE} ARGS ${OCTARG}
#       "${CMAKE_CURRENT_SOURCE_DIR}/lqr.cpp"
#)

# Stuff needed to get DART working

find_package(DART 6 REQUIRED COMPONENTS utils-urdf gui CONFIG)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "lqr.cpp" "lqr.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt config4cpp)
