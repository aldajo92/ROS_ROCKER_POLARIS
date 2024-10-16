cmake_minimum_required(VERSION 3.0.2)
project(pt_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pt_controller
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(path_tracking_controller
  src/PathTrackingController.cpp
)

add_library(planner_path_tracking
  src/PathTrackingPlanner.cpp
)

add_executable(path_tracking_node src/PathTrackingNode.cpp)

add_dependencies(path_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_tracking_node
  planner_path_tracking
  path_tracking_controller
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS})

  catkin_add_gtest(path_tracking_controller_test
    test/test_path_tracking_controller.cpp
  )
  target_link_libraries(path_tracking_controller_test
    path_tracking_controller
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )

  catkin_add_gtest(path_tracking_planner_test
    test/test_path_tracking_planner.cpp
  )
  target_link_libraries(path_tracking_planner_test
    planner_path_tracking
    path_tracking_controller
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
endif()
