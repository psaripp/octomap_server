cmake_minimum_required(VERSION 2.8.3)
project(octomap_server)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        octomap
        octomap_msgs
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if (YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif ()
link_directories(${YAMLCPP_LIBRARY_DIRS})

catkin_package(
        CATKIN_DEPENDS
        roscpp
        octomap_msgs
)

find_package(OCTOMAP REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hxx
        )

add_executable(map_server ${SOURCE_FILES})

add_dependencies(map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server
        ${OCTOMAP_LIBRARIES}
        ${catkin_LIBRARIES}
        )

## Install executables and/or libraries
install(TARGETS map_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE)

