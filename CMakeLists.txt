cmake_minimum_required(VERSION 3.15)
project(LiDAR2BIM VERSION 0.1.0 LANGUAGES CXX)

message("start make LiDAR2BIM")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ")
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/yaml.cmake)
include(cmake/glog.cmake)
include(cmake/pcl.cmake)
include(cmake/openmp.cmake)
include(cmake/global_definition.cmake)
include(cmake/opencv.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)
include_directories(
        ${PROJECT_SOURCE_DIR}/Thirdparty/point-sam/include
)
include_directories(
        ${PROJECT_SOURCE_DIR}/Thirdparty/nanoflann/include
)
file(GLOB_RECURSE ALL_SRCS "src/*.c*" "include/*.h*")
list(APPEND ALL_TARGET_LIBRARIES stdc++fs tbb)
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS} include/global_definition/global_definition.h)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_subdirectory(Thirdparty/point-sam)
add_subdirectory(Thirdparty/backward-cpp)
add_subdirectory(Thirdparty/nanoflann)

target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES}
        POINT_SAM::point_sam
)
message(STATUS "ALL_TARGET_LIBRARIES: ${ALL_TARGET_LIBRARIES}")
add_library(LiDAR2BIM::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# set bin output path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(reg_bm examples/cpp/reg_bm.cpp ${BACKWARD_ENABLE})
add_backward(reg_bm)
target_link_libraries(reg_bm ${PROJECT_NAME})

add_executable(demo_reg examples/cpp/demo_reg.cpp ${BACKWARD_ENABLE})
add_backward(demo_reg)
target_link_libraries(demo_reg ${PROJECT_NAME})