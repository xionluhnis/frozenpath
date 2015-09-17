cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(scanner)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")

add_executable (scanner ${SOURCE} ${HEADERS})
target_link_libraries (scanner ${PCL_LIBRARIES})
