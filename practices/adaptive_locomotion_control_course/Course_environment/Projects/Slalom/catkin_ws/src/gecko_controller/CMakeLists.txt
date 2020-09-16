cmake_minimum_required(VERSION 2.8.3)
project(main_gecko_controller)

set (CMAKE_CXX_STANDARD 11)

SET (SLALOM "../../../../../controllers")
SET (CONTROLLER "${SLALOM}/slalom")
SET (UTILS "${SLALOM}/utils")

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs roscpp )

include_directories("${catkin_INCLUDE_DIRS}"
        "${SLALOM}"
        "${UTILS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
    gecko_controller
	${CONTROLLER}/locomotion_control/main_gecko_controller
	${CONTROLLER}/locomotion_control/Basic_cpg_controller
	${CONTROLLER}/locomotion_control/Pcpg_controller
	${CONTROLLER}/locomotion_control/Delay_line
	${CONTROLLER}/locomotion_control/VRN
	)
target_link_libraries(gecko_controller ${catkin_LIBRARIES})
add_dependencies(gecko_controller ${catkin_EXPORTED_TARGETS})
