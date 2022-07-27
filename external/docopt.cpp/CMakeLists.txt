# ##############################################################################
# docopt.cpp
# ##############################################################################

if(NOT docopt_FOUND)
  download_project(
    PROJ "docopt.cpp"
    GIT_REPOSITORY "https://github.com/docopt/docopt.cpp.git"
    GIT_TAG "v0.6.2"
    UPDATE_DISCONNECTED TRUE)
  add_subdirectory(${docopt.cpp_SOURCE_DIR} ${docopt.cpp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
