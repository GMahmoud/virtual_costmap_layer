################################################################################
## Global declarations

set(PROJECT_DESCRIPTION "${PROJECT_NAME} package")
set(PROJECT_LIB_PREFIX "${PROJECT_NAME}") 
set(CMAKE_PACKAGE_NAME "${PROJECT_LIB_PREFIX}") 
set(CMAKE_PACKAGE_TARGETS "${CMAKE_PACKAGE_NAME}Targets")
set(PROJECT_VERSION_REL_TYPE "${PROJECT_VERSION}-${PROJECT_RELEASE_TYPE}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


################################################################################
## Check release type

list(APPEND RELEASE_TYPE_LIST "alpha" "beta" "release")

list(FIND RELEASE_TYPE_LIST ${PROJECT_RELEASE_TYPE} index)
if(index EQUAL -1)
  message(FATAL_ERROR "Release type '${PROJECT_RELEASE_TYPE}' is incorrect")
endif()


################################################################################
## Compilation outputs paths

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

################################################################################
## CXX declarations

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)