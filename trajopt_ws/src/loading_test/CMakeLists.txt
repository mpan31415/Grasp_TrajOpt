cmake_minimum_required(VERSION 3.5)
project(loading_test)

# Find the Gazebo package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(gazebo 9 REQUIRED)

catkin_package()

# Include the Gazebo headers
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

# Create an executable for get_scene.cpp
add_executable(${PROJECT_NAME}_get_scene_node src/get_scene.cpp)

# Link the executable against the Gazebo libraries
target_link_libraries(
  ${PROJECT_NAME}_get_scene_node ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
  )

# Create a .launch file
install(
  TARGETS ${PROJECT_NAME}_get_scene_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)