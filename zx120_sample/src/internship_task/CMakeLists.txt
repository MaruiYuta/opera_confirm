cmake_minimum_required(VERSION 3.0.2)
project(internship_task)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  roscpp
)

# Actually generate the language-specific message and service files
# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# define executable using MyMessage1 etc.
add_executable(sample_node src/internship_sample.cpp)
add_dependencies(sample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sample_node ${catkin_LIBRARIES})

add_executable(pwri_reproduction_node src/pwri_reproduction.cpp)
add_dependencies(pwri_reproduction_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pwri_reproduction_node ${catkin_LIBRARIES})

add_executable(rosbag_cmd_Publisher src/rosbag_cmd_Publisher.cpp)
add_dependencies(rosbag_cmd_Publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rosbag_cmd_Publisher ${catkin_LIBRARIES})