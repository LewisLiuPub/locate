cmake_minimum_required(VERSION 2.8.3)
project(locate)
set(CMAKE_CXX_STANDARD 11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(
        catkin REQUIRED COMPONENTS cmake_modules geometry_msgs sensor_msgs tf nav_msgs
        roscpp csm laser_simulator csm_wrapper fft_wrapper amcl
)
find_package(Eigen REQUIRED)



if(csm_FOUND)
    # include_directories(${Boost_INCLUDE_DIRS})
    message("\n csm found!!!! \n")
endif()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        DEPENDS Eigen
)
include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS})



add_library(${PROJECT_NAME} src/locate.cpp)
#
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
#




install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)


#### test node

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} )

add_executable( matrix_test include/TinyMatrix/tests/matrix.cpp)
