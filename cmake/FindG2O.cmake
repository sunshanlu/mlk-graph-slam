find_path(G2O_INCLUDE_DIRS g2o/core/base_vertex.h
          "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/include"
          "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/include")

# Find the core elements
find_library(
        G2O_STUFF_LIBRARY
        NAMES g2o_stuff
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")

find_library(
        G2O_CORE_LIBRARY
        NAMES g2o_core
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")

# Find the CLI library
find_library(
        G2O_CLI_LIBRARY
        NAMES g2o_cli
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")

# Find the pluggable solvers
find_library(
        G2O_SOLVER_CHOLMOD
        NAMES g2o_solver_cholmod
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_CSPARSE
        NAMES g2o_solver_csparse
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_CSPARSE_EXTENSION
        NAMES g2o_csparse_extension
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_DENSE
        NAMES g2o_solver_dense
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_PCG
        NAMES g2o_solver_pcg
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_SLAM2D_LINEAR
        NAMES g2o_solver_slam2d_linear
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_STRUCTURE_ONLY
        NAMES g2o_solver_structure_only
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_SOLVER_EIGEN
        NAMES g2o_solver_eigen
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")

# Find the predefined types
find_library(
        G2O_TYPES_DATA
        NAMES g2o_types_data
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_TYPES_ICP
        NAMES g2o_types_icp
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_TYPES_SBA
        NAMES g2o_types_sba
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_TYPES_SCLAM2D
        NAMES g2o_types_sclam2d
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_TYPES_SIM3
        NAMES g2o_types_sim3
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_TYPES_SLAM2D
        NAMES g2o_types_slam2d
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")
find_library(
        G2O_TYPES_SLAM3D
        NAMES g2o_types_slam3d
        PATHS "$ENV{VCPKG_ROOT}/installed/${PLATFORM}/lib"
              "${PROJECT_BINARY_DIR}/vcpkg_installed/${PLATFORM}/lib")

set(G2O_SOLVERS_FOUND "NO")
if(G2O_SOLVER_CHOLMOD
   OR G2O_SOLVER_CSPARSE
   OR G2O_SOLVER_DENSE
   OR G2O_SOLVER_PCG
   OR G2O_SOLVER_SLAM2D_LINEAR
   OR G2O_SOLVER_STRUCTURE_ONLY
   OR G2O_SOLVER_EIGEN)
        set(G2O_SOLVERS_FOUND "YES")
endif(
        G2O_SOLVER_CHOLMOD
        OR G2O_SOLVER_CSPARSE
        OR G2O_SOLVER_DENSE
        OR G2O_SOLVER_PCG
        OR G2O_SOLVER_SLAM2D_LINEAR
        OR G2O_SOLVER_STRUCTURE_ONLY
        OR G2O_SOLVER_EIGEN)

set(G2O_FOUND "NO")
if(G2O_STUFF_LIBRARY
   AND G2O_CORE_LIBRARY
   AND G2O_INCLUDE_DIRS
   AND G2O_SOLVERS_FOUND
   AND PLATFORM)
        set(G2O_FOUND "YES")
endif(
        G2O_STUFF_LIBRARY
        AND G2O_CORE_LIBRARY
        AND G2O_INCLUDE_DIRS
        AND G2O_SOLVERS_FOUND
        AND PLATFORM)
if(G2O_FOUND AND G2O_SOLVERS_FOUND)
        set(G2O_LIBRARIES
            ${G2O_CORE_LIBRARY}
            ${G2O_STUFF_LIBRARY}
            ${G2O_CLI_LIBRARY}
            ${G2O_SOLVER_CHOLMOD}
            ${G2O_SOLVER_CSPARSE}
            ${G2O_SOLVER_CSPARSE_EXTENSION}
            ${G2O_SOLVER_DENSE}
            ${G2O_SOLVER_PCG}
            ${G2O_SOLVER_SLAM2D_LINEAR}
            ${G2O_SOLVER_STRUCTURE_ONLY}
            ${G2O_SOLVER_EIGEN}
            ${G2O_TYPES_DATA}
            ${G2O_TYPES_ICP}
            ${G2O_TYPES_SBA}
            ${G2O_TYPES_SCLAM2D}
            ${G2O_TYPES_SIM3}
            ${G2O_TYPES_SLAM2D}
            ${G2O_TYPES_SLAM3D})
endif(G2O_FOUND AND G2O_SOLVERS_FOUND)
