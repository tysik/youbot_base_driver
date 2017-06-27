cmake_minimum_required(VERSION 2.8.3)
project(youbot_base_driver)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS youbot_driver tf roscpp nodelet nav_msgs geometry_msgs youbot_driver)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  LIBRARIES youbot_base_driver
  CATKIN_DEPENDS youbot_driver tf roscpp nodelet nav_msgs geometry_msgs youbot_driver
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

#
# Build libs
#
add_library(youbot_base_driver src/youbot_base_driver.cpp)
target_link_libraries(youbot_base_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(youbot_base_driver ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(youbot_base_driver_node src/nodes/youbot_base_driver_node.cpp)
target_link_libraries(youbot_base_driver_node youbot_base_driver)

#
# Build nodelets
#
add_library(${PROJECT_NAME}_nodelets src/nodelets/youbot_base_driver_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets youbot_base_driver)

#
# Install libraries
#
install(TARGETS youbot_base_driver ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#
# Install nodes
#
install(TARGETS youbot_base_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#
# Install nodelets plugins description
#
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Install launch files
#
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})