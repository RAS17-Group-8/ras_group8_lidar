cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_lidar)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  image_transport
  rosbag
)

catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf

# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/lidar_node.cpp
  src/Lidar.cpp
)
add_executable(publish_evidence src/publish_evidence.cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(publish_evidence
   ${catkin_LIBRARIES}
 )

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_tests
    test/test_lidar.cpp
    src/Lidar.cpp
  )
  
  target_link_libraries(${PROJECT_NAME}_tests
    ${catkin_LIBRARIES}
  )
endif()

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
