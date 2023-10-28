include_directories("/usr/include/eigen3" "${PROJECT_SOURCE_DIR}/include")

add_executable(
        kittiset_test
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/config.cc"
        "kittiset_test.cc")
target_link_libraries(kittiset_test ${DEPEND_LIBS})

add_executable(
        triangulate_test
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/config.cc"
        "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
        "triangulate_test.cc")
target_link_libraries(triangulate_test ${DEPEND_LIBS})

add_executable(
        frontend_test
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/viewer.cc"
        "${PROJECT_SOURCE_DIR}/src/config.cc"
        "frontend_test.cc")
target_link_libraries(frontend_test ${DEPEND_LIBS})

add_executable(
        viewer_test
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/viewer.cc"
        "${PROJECT_SOURCE_DIR}/src/config.cc"
        "viewer_test.cc")
target_link_libraries(viewer_test ${DEPEND_LIBS})

add_executable(
        backend_test
        "${PROJECT_SOURCE_DIR}/src/backend.cc"
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/viewer.cc"
        "${PROJECT_SOURCE_DIR}/src/config.cc"
        "backend_test.cc")
target_link_libraries(backend_test ${DEPEND_LIBS})
