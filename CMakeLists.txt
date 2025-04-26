cmake_minimum_required(VERSION 3.0.2)
project(freedom)

ADD_COMPILE_OPTIONS(-std=c++17 )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# SET(CMAKE_BUILD_TYPE "Debug")  
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")  
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(rosconsole REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  roscpp
  message_generation
  std_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  rosbag
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs rosbag
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable( freedom_node
  src/node.cpp
  src/freenode.cpp
  src/freedom.cpp
  src/visualization.cpp
  src/map.cpp
  src/scanmap.cpp
  src/mrmap.cpp
  src/raycast.cpp
  src/depth_image.cpp
)
target_link_libraries( freedom_node
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${cv_bridge_LIBRARIES}
)

add_executable( ground_truth_generate
  src/evaluate_tools/ground_truth_generate.cpp
)
target_link_libraries( ground_truth_generate
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)

add_executable( static_map_evaluate
  src/evaluate_tools/static_map_evaluate.cpp
)
target_link_libraries( static_map_evaluate
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)