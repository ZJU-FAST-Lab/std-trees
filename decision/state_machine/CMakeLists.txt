cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    occ_grid
    kino_plan
    r3_plan
    poly_opt
    quadrotor_msgs
    visualization_utils
    self_msgs_and_srvs
    bspline_opt
    bspline
    sampler
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES state_machine
    CATKIN_DEPENDS occ_grid sampler
#     DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node 
#     src/state_machine_node.cpp
#     src/fsm.cpp
# )
# target_link_libraries(${PROJECT_NAME}_node
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )

add_executable(traj_server_node
    src/traj_server_node.cpp
)
target_link_libraries(traj_server_node
    ${catkin_LIBRARIES}
)

#add_executable(pub_goal_node
#    src/pub_goal_node.cpp
#)
#target_link_libraries(pub_goal_node
#    ${catkin_LIBRARIES}
#)

add_executable(test_bvp_node
    src/test_bvp_node.cpp
)
target_link_libraries(test_bvp_node
    ${catkin_LIBRARIES}
)

add_executable(test_planner_node
    src/test_planners_node.cpp
)
target_link_libraries(test_planner_node
    ${catkin_LIBRARIES}
)

# add_executable(global_trajs_node
#     src/global_trajs_node.cpp
# )
# target_link_libraries(global_trajs_node
#     ${catkin_LIBRARIES}
# )

add_dependencies(
  ${catkin_EXPORTED_TARGETS}
)

