# ##############################################################################
# Version setup
# ##############################################################################

file(STRINGS ${CMAKE_SOURCE_DIR}/VERSION PROJECT_VERSION)
message(STATUS "Project version: ${PROJECT_VERSION}")

# ##############################################################################
# Project information
# ##############################################################################

cmake_minimum_required(VERSION 3.5.1)
project(virtual_costmap_layer LANGUAGES C CXX VERSION ${PROJECT_VERSION})
set(PROJECT_RELEASE_TYPE release) # alpha, beta, release

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(declarations)
include(global)
include(debug)
include(dependencies)

# ##############################################################################
# Build external project
# ##############################################################################

add_subdirectory(external)

# ##############################################################################
# Version file configuration
# ##############################################################################

configure_file(include/${PROJECT_NAME}/version.hpp.in
               ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}_version.hpp @ONLY)

# ##############################################################################
# Load and setup packages
# ##############################################################################

find_package(catkin REQUIRED
             COMPONENTS
             message_generation
             roscpp
             costmap_2d
             dynamic_reconfigure
             tf
             geometry_msgs
             std_srvs)

# ##############################################################################
# Generate messages and configurations
# ##############################################################################

generate_dynamic_reconfigure_options(cfg/VirtualLayer.cfg)

add_message_files(FILES
                  Circle.msg
                  Form.msg
                  LineString.msg
                  Polygon.msg
                  Ring.msg)

add_service_files(FILES
                  AddElement.srv
                  GetElement.srv
                  GetElements.srv
                  RemoveElement.srv)

generate_messages(DEPENDENCIES
                  geometry_msgs)

# ##############################################################################
# Setup catkin packages
# ##############################################################################

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               virtual_layer
               CATKIN_DEPENDS
               message_runtime
               costmap_2d
               dynamic_reconfigure
               tf
               geometry_msgs
               std_srvs)

# ##############################################################################
# Generate library and executables
# ##############################################################################

include_directories(${CMAKE_BINARY_DIR}/include ${catkin_INCLUDE_DIRS} include)

add_library(virtual_layer src/virtual_layer.cpp)
target_link_libraries(virtual_layer ${catkin_LIBRARIES} Boost::boost)
add_dependencies(virtual_layer ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(interface src/version.cpp)
target_link_libraries(interface PRIVATE docopt_s nlohmann_json::nlohmann_json)

# ##############################################################################
# Install targets
# ##############################################################################

install(TARGETS interface RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS virtual_layer LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# ##############################################################################
# Install headers
# ##############################################################################

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp")

# ##############################################################################
# Install files and configurations
# ##############################################################################

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING
        PATTERN "*.launch")

install(DIRECTORY cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
        FILES_MATCHING
        PATTERN "*.cfg")

install(DIRECTORY cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
        FILES_MATCHING
        PATTERN "banner")
