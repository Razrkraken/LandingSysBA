cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(rodos_plugin CXX C)
set(CMAKE_CXX_STANDARD 14)

# Find Rodos
set(ENABLE_RODOS_RTTI ON CACHE BOOL "Enable run-time type information for RODOS" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(deps/rodos/cmake/port/posix.cmake)
add_subdirectory(deps/rodos)

# Find Gazebo
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -Wl,--no-undefined")

# Find threading Library
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

# Build rodos_plugin
add_library(rodos_plugin SHARED src/rodos_plugin.cpp include/GazeboTopic.h include/rodos_plugin.h)
target_link_libraries(rodos_plugin PUBLIC ${GAZEBO_LIBRARIES} rodos Threads::Threads ${CMAKE_DL_LIBS})
target_include_directories(rodos_plugin PUBLIC include)

set(BUILD_EXAMPLE OFF)
#
if (BUILD_EXAMPLE)
    # Example
    add_library(cessna SHARED example/src/cessna.cpp)
    target_link_libraries(cessna ${GAZEBO_LIBRARIES})
    add_library(example SHARED example/src/example.cpp)
    target_link_libraries(example rodos_plugin cessna)
    add_custom_target(
            run_example
            COMMAND ${CMAKE_COMMAND} -E env GAZEBO_MODEL_PATH=$ENV{GAZEBO_MODEL_PATH}:${CMAKE_CURRENT_SOURCE_DIR}/example/world ; GAZEBO_RESOURCE_PATH=$ENV{GAZEBO_RESOURCE_PATH}:${CMAKE_CURRENT_SOURCE_DIR}/example/world:
            gazebo --verbose --pause example/world/cessna_demo.world
            DEPENDS example
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

# Activate this to run Glider Simulation
set(BUILD_GAZEBO_RODOS_PLUGIN_GLIDER ON)

if (BUILD_GAZEBO_RODOS_PLUGIN_GLIDER)

    add_library(glider SHARED ../Glider/src/modelGliderPlugin.cpp)
    target_link_libraries(glider ${GAZEBO_LIBRARIES})
    add_library(rodosGlider SHARED ../Glider/src/rodosGlider.cpp)
    target_link_libraries(rodosGlider rodos_plugin glider)

    add_custom_target(
            run_glider
            COMMAND ${CMAKE_COMMAND} -E env GAZEBO_MODEL_PATH=$ENV{GAZEBO_MODEL_PATH}:/home/alejandro/gazebo_ws/GliderBA/Glider/world ; GAZEBO_RESOURCE_PATH=$ENV{GAZEBO_RESOURCE_PATH}:/home/alejandro/gazebo_ws/GliderBA/Glider/world
            gazebo --verbose --pause --physics ode ../Glider/world/glider.world
            DEPENDS rodosGlider
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

endif()
