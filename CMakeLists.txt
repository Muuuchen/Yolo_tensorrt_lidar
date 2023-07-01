cmake_minimum_required(VERSION 2.6)

project(yolov5)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(CUDA REQUIRED)

if(WIN32)
enable_language(CUDA)
endif(WIN32)

include_directories(${PROJECT_SOURCE_DIR}/include)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(realsense2 REQUIRED)

include_directories(${realsense2_INCLUDE_DIRS})
add_library(serial_shared ${PROJECT_SOURCE_DIR}/SerialInterface.cpp)
add_library(UDPSocket_shared ${PROJECT_SOURCE_DIR}/UDPSocket.cpp)
add_library(TCPSocket_shared ${PROJECT_SOURCE_DIR}/TCPSocket.cpp)
include_directories(../)

add_executable(
    yolov5 
    ${PROJECT_SOURCE_DIR}/calibrator.cpp 
    ${PROJECT_SOURCE_DIR}/yolov5_trt.cpp 
    sort-cpp-master/sort-c++/KalmanTracker.cpp
    sort-cpp-master/sort-c++/Hungarian.cpp
    ./_track.cpp
    ) 

#add_executable(yolov5 ${PROJECT_SOURCE_DIR}/calibrator.cpp ${PROJECT_SOURCE_DIR}/rsTest.cpp) 

target_link_libraries(yolov5 nvinfer)
target_link_libraries(yolov5 cudart)
target_link_libraries(yolov5 serial_shared)
target_link_libraries(yolov5 UDPSocket_shared)
target_link_libraries(yolov5 TCPSocket_shared)
target_link_libraries(yolov5 myplugins)
target_link_libraries(yolov5 ${OpenCV_LIBS})
target_link_libraries(yolov5 ${realsense2_LIBRARY})
if(UNIX)
add_definitions(-O3 -pthread)
endif(UNIX)

