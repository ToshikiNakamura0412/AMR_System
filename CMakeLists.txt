cmake_minimum_required(VERSION 3.0.2)
project(second_challenge_toshiki)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf2
  roomba_500driver_meiji
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES second_challenge_toshiki
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME} src/${PROJECT_NAME}/second_challenge_toshiki.cpp)
# add_dependencies(global_path_planner src/global_path_planner.cpp)

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(global_path_planner src/global_path_planner_node.cpp src/global_path_planner.cpp)
add_executable(global_path_planner src/global_path_planner_node.cpp src/global_path_planner.cpp)
add_executable(localizer           src/localizer_node.cpp           src/localizer.cpp)
add_executable(local_goal_creator  src/local_goal_creator_node.cpp  src/local_goal_creator.cpp)
add_executable(local_map_creator   src/local_map_creator_node.cpp   src/local_map_creator.cpp)
add_executable(local_path_planner  src/local_path_planner_node.cpp  src/local_path_planner.cpp)
add_executable(obstacle_expander   src/obstacle_expander_node.cpp   src/obstacle_expander.cpp)
add_executable(obstacle_detector   src/obstacle_detector_node.cpp   src/obstacle_detector.cpp)

## Add cmake target dependencies of the executable
add_dependencies(global_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(localizer           ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_goal_creator  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_map_creator   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_path_planner  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_expander   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_detector   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(global_path_planner ${catkin_LIBRARIES})
target_link_libraries(localizer           ${catkin_LIBRARIES})
target_link_libraries(local_goal_creator  ${catkin_LIBRARIES})
target_link_libraries(local_map_creator   ${catkin_LIBRARIES})
target_link_libraries(local_path_planner  ${catkin_LIBRARIES})
target_link_libraries(obstacle_expander   ${catkin_LIBRARIES})
target_link_libraries(obstacle_detector   ${catkin_LIBRARIES})
