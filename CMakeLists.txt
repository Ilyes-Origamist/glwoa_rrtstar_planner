cmake_minimum_required(VERSION 3.0.2)
project(glwoa_rrtstar_planner)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## find catkin required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  nav_core
  base_local_planner
  geometry_msgs
  dynamic_reconfigure
  actionlib
  move_base_msgs
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/GlWoaRrtStarPlanner.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    pluginlib
    nav_core
    base_local_planner
    geometry_msgs
    dynamic_reconfigure
    actionlib
    move_base_msgs
)
 

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/glwoa_rrtstar_planner.cpp # global planner
  src/rrt_star.cpp # rrt star algorithm
  src/woa_agent.cpp # path optimizer (Whale Optimization Algorithm)
  src/collision_detector.cpp # collision detection
  src/random_double_generator.cpp # for RRT* sampling
  src/random_int_generator.cpp # for WOA sampling
)

add_executable(send_goal src/send_goal.cpp)
target_link_libraries(send_goal ${catkin_LIBRARIES})
add_dependencies(send_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# plugin xml for registering the planner as nav_core::BaseGlobalPlanner
install(
  FILES glwoa_rrtstar_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/glwoa_rrtstar_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



## unit tests
# if (CATKIN_ENABLE_TESTING)
#   find_package(cmake_modules)
#   enable_testing()
#   find_package(rostest REQUIRED)

#   include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#   )

  # add_rostest_gtest(test_rrt_star
  #   test/rrt_star.test
  #   test/src/test_rrt_star.cpp
  # )

  # target_link_libraries(test_rrt_star
  #   ${catkin_LIBRARIES}
  #   ${PROJECT_NAME}
  #   gtest
  # )

# endif()
