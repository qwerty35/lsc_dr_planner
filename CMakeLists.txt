cmake_minimum_required(VERSION 3.16)
project(lsc_dr_planner)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set(CMAKE_CXX_STANDARD 17)
include_directories(${PROJECT_SOURCE_DIR}/include)

#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#CPLEX
add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio221)
set(CPLEX_INCLUDE_DIR     ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR   ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR   ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})
set(CPLEX_LIBRARIES
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)

#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
  pcl_ros
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime octomap_ros octomap_msgs pcl_ros
  DEPENDS OCTOMAP
)

#openGJK
set(OPENGJK_SRC
  src/openGJK/openGJK.cpp
)

#MAPF
add_subdirectory(third_party/grid-pathfinding/graph ./graph)
set(MAPF_SRC
  src/mapf/lib_cbs.cpp
  src/mapf/solver.cpp
  src/mapf/paths.cpp
  src/mapf/plan.cpp
  src/mapf/problem.cpp
  src/mapf/pibt.cpp
  src/mapf/ecbs.cpp
)

#LSC planner
set(LSC_PLANNER_SRC
  src/param.cpp
  src/mission.cpp
  src/trajectory.cpp
  src/agent_manager.cpp
  src/map_manager.cpp
  src/traj_planner.cpp
  src/traj_optimizer.cpp
  src/goal_optimizer.cpp
  src/grid_based_planner.cpp
  src/collision_constraints.cpp
  src/linear_kalman_filter.cpp
  ${OPENGJK_SRC}
)

#########
# Build #
#########
add_executable(multi_sync_simulator_node
  src/multi_sync_simulator_node.cpp
  src/multi_sync_simulator.cpp
  src/multi_sync_replayer.cpp
  ${LSC_PLANNER_SRC}
  ${MAPF_SRC}
)
add_dependencies(multi_sync_simulator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_sync_simulator_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${CPLEX_LIBRARIES}
  ${PCL_LIBRARIES}
  lib-graph
  stdc++fs
)

