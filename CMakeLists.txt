cmake_minimum_required(VERSION 3.15)

project(mlk-slam CXX)
set(DEPEND_LIBS depend_libs)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(test)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("/usr/include/eigen3")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(G2O_LIBS g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse
             g2o_csparse_extension)

file(GLOB SRC_FILES "${PROJECT_SOURCE_DIR}/src/*.cc")

find_package(OpenCV 3.1 REQUIRED)
find_package(Sophus REQUIRED)
find_package(FMT REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(spdlog REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp REQUIRED IMPORTED_TARGET)

add_library(${DEPEND_LIBS} INTERFACE)
target_include_directories(
        ${DEPEND_LIBS}
        INTERFACE ${OpenCV_INCLUDE_DIRS}
        INTERFACE ${Pangolin_INCLUDE_DIRS}
        INTERFACE ${Sophus_INCLUDE_DIRS}
        INTERFACE ${G2O_INCLUDE_DIRS}
        INTERFACE ${FMT_INCLUDE_DIRS})
target_link_libraries(
        ${DEPEND_LIBS}
        INTERFACE ${OpenCV_LIBS}
        INTERFACE ${Sophus_LIBRARIES}
        INTERFACE ${Pangolin_LIBRARIES}
        # INTERFACE fmt::fmt
        INTERFACE spdlog::spdlog
        INTERFACE ${G2O_LIBS}
        INTERFACE ${GFLAGS_LIBRARIES}
        INTERFACE ${GLOG_LIBRARIES}
        INTERFACE PkgConfig::JSONCPP)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${DEPEND_LIBS})
