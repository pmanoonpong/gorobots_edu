cmake_minimum_required(VERSION 2.8.3)
project(keyboard)

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
        keyboard
        ${CONTROLLER}/keyboard/keyboard
	)
target_link_libraries(keyboard ${catkin_LIBRARIES})
add_dependencies(keyboard ${catkin_EXPORTED_TARGETS})
