cmake_minimum_required(VERSION 2.8.3)
project(quadcopter_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

#generate_messages(
 # DEPENDENCIES
  # std_msgs
 #)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(quad_test src/flight_test.cpp src/main.cpp)
add_dependencies(quad_test quadcopter_test_generate_messages_cpp)
target_link_libraries(quad_test ${catkin_LIBRARIES})

add_executable(vector_test src/vector_calc.cpp src/vector_main.cpp)
add_dependencies(vector_test quadcopter_test_generate_messages_cpp)
target_link_libraries(vector_test ${catkin_LIBRARIES})

add_executable(Tap_Decisions src/Tap_Decision.cpp src/Tap_main.cpp)
add_dependencies(Tap_Decisions quadcopter_test_generate_messages_cpp)
target_link_libraries(Tap_Decisions ${catkin_LIBRARIES})




#add_executable(main_test src/flight_test.cpp)
#add_dependencies(main_test quadcopter_test_generate_messages_cpp)
#target_link_libraries(main_test ${catkin_LIBRARIES})
