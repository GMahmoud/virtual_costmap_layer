# ##############################################################################
# System information
# ##############################################################################

set(HOST_SYSTEM "${CMAKE_HOST_SYSTEM_PROCESSOR}-${CMAKE_HOST_SYSTEM}")
set(TARGET_SYSTEM "${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM}")
string(TIMESTAMP PROJECT_BUILD_DATE "%Y-%m-%d %H:%M:%S utc" UTC)

# ##############################################################################
# Repository information
# ##############################################################################
find_package(Git)

if(Git_FOUND)
  # Git unstaged changes
  execute_process(
    COMMAND bash -c "${GIT_EXECUTABLE} -C ${PROJECT_SOURCE_DIR} status -s -uall"
    OUTPUT_VARIABLE GIT_CHANGES_OUTPUT)
  string(REGEX
         REPLACE "\n$"
                 ""
                 GIT_CHANGES_OUTPUT
                 "${GIT_CHANGES_OUTPUT}")

  if(GIT_CHANGES_OUTPUT STREQUAL "")
    set(GIT_UNSTAGED_CHANGES 0)
  else()
    set(GIT_UNSTAGED_CHANGES 1)
  endif()

  # Git branch
  execute_process(
    COMMAND
      bash -c
      "${GIT_EXECUTABLE} -C ${PROJECT_SOURCE_DIR} rev-parse --abbrev-ref HEAD"
    OUTPUT_VARIABLE GIT_BRANCH)
  string(REGEX
         REPLACE "\n$"
                 ""
                 GIT_BRANCH
                 "${GIT_BRANCH}")

  # Git commit
  execute_process(
    COMMAND bash -c "${GIT_EXECUTABLE} -C ${PROJECT_SOURCE_DIR} rev-parse HEAD"
    OUTPUT_VARIABLE GIT_COMMIT)
  string(REGEX
         REPLACE "\n$"
                 ""
                 GIT_COMMIT
                 "${GIT_COMMIT}")

else()
  set(GIT_UNSTAGED_CHANGES 0)
endif()

# ##############################################################################
# Custom functions
# ##############################################################################

## This function is used to find Boost by checking suitable versions
function(find_boost)
  set(proper_boost_versions 1.75 1.74 1.73 1.72 1.71 1.69 1.65.1)
  foreach(proper_boost_version ${proper_boost_versions})
    find_package(Boost ${proper_boost_version} EXACT QUIET)
    if(Boost_FOUND)
      set(BOOST_VER ${proper_boost_version} CACHE STRING "boost version")
      return()
    endif()
  endforeach()
  message(FATAL_ERROR "CMake could not find a proper version of Boost (${proper_boost_versions})")
endfunction()