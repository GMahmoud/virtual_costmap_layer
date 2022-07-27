# ##############################################################################
# nlohmann_json
# ##############################################################################

if(NOT nlohmann_json_FOUND)
  download_project(
    PROJ "nlohmann_json"
    GIT_REPOSITORY "https://github.com/nlohmann/json.git"
    GIT_TAG "v3.9.1"
    UPDATE_DISCONNECTED TRUE)
  add_subdirectory(${nlohmann_json_SOURCE_DIR} ${nlohmann_json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
