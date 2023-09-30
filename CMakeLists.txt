cmake_minimum_required(VERSION 3.15)
project(myslam)

add_subdirectory(test)
set(CMAKE_BUILD_TYPE "Release")

set(G2O_LIBRARIES g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)
set(DEPEND_LIB depend_lib)
file(GLOB SRC_FILES src/*.cpp)

include_directories(include)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(OpenCV 3.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)
find_package(fmt REQUIRED)
find_package(Glog REQUIRED)

add_library(${DEPEND_LIB} INTERFACE)
target_link_libraries(
        ${DEPEND_LIB}
        INTERFACE ${OpenCV_LIBS}
        INTERFACE Sophus::Sophus
        INTERFACE fmt::fmt
        INTERFACE ${Pangolin_LIBRARIES}
        INTERFACE ${G2O_LIBRARIES}
        INTERFACE ${GLOG_LIBRARIES}
)
target_include_directories(
        ${DEPEND_LIB}
        INTERFACE ${OpenCV_INCLUDE_DIRS}
        INTERFACE ${Sophus_INCLUDE_DIRS}
        INTERFACE ${Pangolin_INCLUDE_DIRS}
        INTERFACE ${EIGEN3_INCLUDE_DIRS}
        INTERFACE ${G2O_INCLUDE_DIRS}
)

message(STATUS OPENCV_INCLUDE_DIRS--${OpenCV_INCLUDE_DIRS})
message(STATUS Sophus_INCLUDE_DIRS--${Sophus_INCLUDE_DIRS})
message(STATUS Pangolin_INCLUDE_DIRS--${Pangolin_INCLUDE_DIRS})
message(STATUS EIGEN3_INCLUDE_DIRS--${EIGEN3_INCLUDE_DIRS})
message(STATUS G2O_LIBRARIES -- ${G2O_LIBRARIES})
message(STATUS G2O_INCLUDE_DIRS -- ${G2O_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(
        ${PROJECT_NAME}
        ${DEPEND_LIB}
)