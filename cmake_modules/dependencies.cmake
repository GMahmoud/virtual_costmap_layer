# ##############################################################################
# Dependencies
# ##############################################################################

find_boost()

find_package(docopt)
find_package(nlohmann_json)
find_package(Boost ${BOOST_VER} REQUIRED)
