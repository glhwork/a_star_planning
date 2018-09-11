cmake_minimum_required(VERSION 2.8.3)
project(a_star_planning)

add_compile_options(-std=c++11)

include_directories(
  include
 )

add_library(astar
  src/AStar.cpp 
)

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main
  astar
)
