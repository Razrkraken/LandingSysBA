cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(gazebo_rodos_plugin_example CXX C)
set(CMAKE_CXX_STANDARD 14)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -Wl,--no-undefined")

# Add a Gazebo library that handles messages from RODOS topics
add_library(cessna SHARED src/cessna.cpp)
target_link_libraries(cessna ${GAZEBO_LIBRARIES})


# Include the Gazebo RODOS plugin
add_subdirectory(path/to/Gazebo_rodos_plugin)

# Add your code that uses RODOS topics
add_library(example SHARED src/example.cpp)
# And link with the rodos_plugin
target_link_libraries(example rodos_plugin cessna)

# This target can be used to start Gazebo with the example world
add_custom_target(
        run_example
        COMMAND ${CMAKE_COMMAND} -E env GAZEBO_MODEL_PATH=$ENV{GAZEBO_MODEL_PATH}${CMAKE_CURRENT_SOURCE_DIR}/world: GAZEBO_RESOURCE_PATH=$ENV{GAZEBO_RESOURCE_PATH}${CMAKE_CURRENT_SOURCE_DIR}/world:
        gazebo --verbose world/cessna_demo.world
        DEPENDS example
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
