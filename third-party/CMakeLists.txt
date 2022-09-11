# ##############################################################################
# Generate library and executables
# ##############################################################################

set(LIBRARY_NAME third-party)

file(GLOB_RECURSE SOURCES *.cpp *.c *.cc)
file(GLOB_RECURSE HEADERS *.hpp *.ipp *.h)

add_library(${LIBRARY_NAME} ${SOURCES} ${HEADERS})
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# ##############################################################################
# Install targets
# ##############################################################################

install(TARGETS ${LIBRARY_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
