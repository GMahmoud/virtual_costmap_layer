cmake_minimum_required(VERSION 2.8.3)
project(virtual_costmap_layer)

find_package(catkin REQUIRED
	roscpp
  costmap_2d
  dynamic_reconfigure
  custom_msgs
  tf
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

generate_dynamic_reconfigure_options(
  cfg/VirtualLayer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES VirtualLayer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure custom_msgs tf
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(VirtualLayer src/VirtualLayer.cpp)
target_link_libraries(VirtualLayer ${catkin_LIBRARIES})
 
add_dependencies(VirtualLayer ${PROJECT_NAME}_gencfg)




