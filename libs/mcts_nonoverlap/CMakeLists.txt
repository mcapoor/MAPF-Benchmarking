cmake_minimum_required(VERSION 3.10...3.31)

project(mcts_nonoverlap)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_executable(mcts_nonoverlap
  mcts_nonoverlap.cpp
)
target_include_directories(mcts_nonoverlap PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(mcts_nonoverlap
  ${Boost_LIBRARIES}
  yaml-cpp
)
