cmake_minimum_required(VERSION 2.8.3)
project(map_server)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_msgs
            rostest
        )

find_package(Boost REQUIRED COMPONENTS system)

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        image_loader
    CATKIN_DEPENDS
        roscpp
        tf
        nav_msgs
)
#Show header files in IDE
file(GLOB_RECURSE library_header_files ${BASE_DIR}/include/
*.h *.hpp)
add_custom_target( library_headers SOURCES ${library_header_files} )

include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

# Library for loading map image
add_library(map_image_loader src/map_image_loader/map_image_loader.cpp)
target_link_libraries(map_image_loader SDL SDL_image ${Boost_LIBRARIES})

# Executable for serving a map
add_executable(map_server
    src/main_map_server.cpp
    src/map_server/map_server.cpp
)

target_link_libraries(map_server
    map_image_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

# Executable for saving a map
add_executable(map_saver
    src/main_map_saver.cpp
    src/map_saver/map_saver.cpp
)

target_link_libraries(map_saver
    ${catkin_LIBRARIES}
)

## Install executables and/or libraries
install(TARGETS 
map_saver 
map_server 
map_image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
