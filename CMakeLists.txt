cmake_minimum_required(VERSION 2.8.3)
project(robotnik_modbus_io)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    self_test
    diagnostic_updater
    std_srvs
    robotnik_msgs
    std_msgs    
)


#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    self_test
    diagnostic_updater
    std_srvs
    robotnik_msgs
#  DEPENDS system_lib
)

include_directories(
	include
	/usr/include/modbus
	${catkin_INCLUDE_DIRS}
)

add_executable(robotnik_modbus_io_node src/robotnik_modbus_io_node.cpp)

add_dependencies(robotnik_modbus_io_node robotnik_msgs_generate_messages_cpp)

target_link_libraries(
	robotnik_modbus_io_node
	modbus
	${catkin_LIBRARIES}
)

install(TARGETS robotnik_modbus_io_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
