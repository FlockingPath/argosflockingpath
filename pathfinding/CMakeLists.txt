cmake_minimum_required(VERSION 3.10)
project(Pathfinding)

set(CMAKE_CXX_STANDARD 11)

add_executable(test_dijkstra test_dijkstra.cpp dijkstra.cpp)