cmake_minimum_required(VERSION 2.8.3)
project(sentinel)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -std=c++0x -std=c++14 -fexceptions -Wno-unused-local-typedefs")
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    map_server
    geometry_msgs
    nav_msgs
    sensor_msgs
    rospy
    std_msgs
    pcl_ros
    tf
    livox_ros_driver
    actionlib
    move_base_msgs
    std_srvs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs actionlib move_base_msgs std_srvs
    DEPENDS EIGEN3 PCL OpenCV
)
include_directories(${PROJECT_SOURCE_DIR}/include/serial/include)


add_library(serial
    ${PROJECT_SOURCE_DIR}/include/serial/src/impl/unix.cc
    ${PROJECT_SOURCE_DIR}/include/serial/src/serial.cc
    ${PROJECT_SOURCE_DIR}/include/serial/src/impl/list_ports/list_ports_linux.cc
)
add_library(voronoi
    ${PROJECT_SOURCE_DIR}/include/dynamic_voronoi/dynamicvoronoi.cpp
)
add_library(resources
    ${PROJECT_SOURCE_DIR}/src/resources/resources.cpp
)
add_library(dm
    ${PROJECT_SOURCE_DIR}/src/DistanceManager/DistanceManager.cpp
)
add_library(car_status
    ${PROJECT_SOURCE_DIR}/src/CarStatus/CarStatus.cpp
)

add_executable(voronoiPlanner src/Planner/VoronoiPlan.cpp)
target_link_libraries(voronoiPlanner voronoi resources dm car_status ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(minCostPlanner src/Planner/MinCostPlan.cpp)
target_link_libraries(minCostPlanner voronoi resources dm car_status ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(sentinel 
    src/rmSerial/Sentinel.cpp
    src/rmSerial/rmSerial.cpp)
target_link_libraries(sentinel resources serial car_status ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )


# add_executable(chanpcl src/livox_repub.cpp)
# target_link_libraries(chanpcl ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})





# add_executable(scantomap src/scanTomap.cpp)
# target_link_libraries(scantomap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})