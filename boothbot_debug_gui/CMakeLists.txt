### This is a default CMakeLists.txt template that you can use for your project
### It is based on http://wiki.ros.org/catkin/CMakeLists.txt
### You can always remove or comment out stuff you don't need

#######################
## Compiler settings ##
#######################

## Add minimum required cmake version
cmake_minimum_required(VERSION 3.1)
## Project name should match the name of the package you created
project(boothbot_debug_gui)
set(CMAKE_CXX_STANDARD 11)
## CMake flags not used in current stages.
# add_definitions("-Wall")
# add_definitions("-Wextra")
# add_definitions("-Wpedantic")
# add_definitions("-Werror")

## Custom options
# set(MAKE_SHARED False CACHE BOOL "Build and use shared libraries")
# if(NOT MAKE_SHARED)
#     set(CMAKE_FIND_LIBRARY_PREFIXES ".a")
#     set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
#     set(BUILD_SHARED_LIBS OFF)
# endif()

##############
## Packages ##
##############

## Find catkin
find_package(catkin REQUIRED)
# Find 3rd packages
find_package(Curses REQUIRED)
# find_package(PCL 1.7 REQUIRED)
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    boothbot_msgs
    augustbot_msgs
    message_filters
    nav_msgs
    sensor_msgs
)
# Find other packages
find_package(catkin REQUIRED COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
)

####################
## Catkin package ##
####################
## Catkin package definition
# INCLUDE_DIRS: which directories should be included
# LIBRARIES: which libraries should be linked
# CATKIN_DEPENDS: This uses the above definition of ${PROJECT}_CATKIN_DEPS
# DEPENDS: OTHER 3RD party libraries that your package requires
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    DEPENDS
)
###########
## Build ##
###########
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Add project source files
set(PROJECT_SOURCE
    src/ros_node.cpp
    src/ros_boothbot_debug_gui.cpp
    src/ncurses_utils.cpp
)
# Make definition of project libs
# if your package needs some library, you can define here
# catkin libraries is needed here
set(PROJECT_LIBS
    pthread
    atomic
    ncurses
    ${catkin_LIBRARIES}
)

# If you are making some shared libraries this will word
# if(MAKE_SHARED)
#     add_library(${PROJECT_NAME} ${PROJECT_SOURCE})
# else()
#     add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCE})
# endif()

# Executables and target dependencies
add_executable(boothbot_debug_gui
    src/ros_boothbot_debug_gui.cpp
    src/ros_node.cpp
    src/ncurses_utils.cpp
    ${PROJECT_SOURCE}
)
# Avoid compile failure
add_dependencies(boothbot_debug_gui ${boothbot_debug_gui_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(boothbot_debug_gui ${PRJECT_NAME} ${PROJECT_LIBS})

#############
## Install ##
#############

install(
    TARGETS
    boothbot_debug_gui
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    config
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
