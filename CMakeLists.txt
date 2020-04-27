cmake_minimum_required(VERSION 2.8.3)
project(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set( CMAKE_BUILD_TYPE "Release" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )


# YAML library
FIND_PACKAGE(yaml-cpp REQUIRED CONFIG PATHS ${YAML_PATHS})
# 添加Pangolin依赖
find_package( Pangolin )
find_package( Glog REQUIRED)
find_package( Gflags REQUIRED)
find_package( OpenCV 3.1 REQUIRED )
find_package( Sophus REQUIRED )
find_package(PCL REQUIRED QUIET)
include_directories( 
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${GLOG_INCLUDE_DIR}
    "/usr/local/include/eigen3/"
    ${PROJECT_SOURCE_DIR}/include
    ${YAML_CPP_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
#     /media/zhijian/Document/grow/slam/PL_SLAM/pl_slam/include
#     /media/zhijian/Document/grow/slam/PL_SLAM/pl_slam/3rdparty/line_descriptor/include
#     /media/zhijian/Document/grow/slam/PL_SLAM/pl_slam/3rdparty/line_descriptor
    )

message("\r\n\r\n")
message(${PCL_INCLUDE_DIRS})
message(${PCL_LIBRARY_DIRS})
message(${PCL_LIBRARIES})
    
link_directories(
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)

# add_definitions(${PCL_DEFINITIONS})
list(APPEND SOURCEFILES
  src/findpoint.cpp
  src/file.cpp
  src/rotate.cpp
  src/useSophus.cpp
  src/learncv.cpp
  src/pcltest.cpp
  src/removeGround.cpp
  src/utility.cpp
)
add_library(srclib SHARED ${SOURCEFILES})


add_executable(test main.cpp)
target_link_libraries( test ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${GLOG_LIBRARIES} ${PCL_LIBRARIES} srclib)
target_link_libraries(test ${YAML_CPP_LIBRARIES} ${Sophus_LIBRARIES})


