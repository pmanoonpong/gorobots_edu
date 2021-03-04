cmake_minimum_required(VERSION 2.8.3)
project(main_pendulum_controller)

set (CMAKE_CXX_STANDARD 11)

SET (GOROBOTS "../../../../..")
SET (CONTROLLER "${GOROBOTS}/controllers")
SET (UTILS "${GOROBOTS}/utils")

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs roscpp)

include_directories("${catkin_INCLUDE_DIRS}"
        "${GOROBOTS}"
        "${UTILS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
        pendulum_controller
        ${CONTROLLER}/pendulum/main_pendulum_controller
        ${CONTROLLER}/pendulum/simRosClass
        ${CONTROLLER}/pendulum/pendulumController )

target_link_libraries(pendulum_controller ${catkin_LIBRARIES})
add_dependencies(pendulum_controller ${catkin_EXPORTED_TARGETS})
