include_directories("/usr/include/eigen3" "${PROJECT_SOURCE_DIR}/include")

add_executable(eigen_block_test
               "${CMAKE_CURRENT_SOURCE_DIR}/eigen_block_test.cc")

add_executable(
        kittiset_test
        "${CMAKE_CURRENT_SOURCE_DIR}/kittiset_test.cc"
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc")
target_link_libraries(kittiset_test ${DEPEND_LIBS})

add_executable(so3d_test "so3d_test.cc")
target_link_libraries(so3d_test ${DEPEND_LIBS})

add_executable(eigen_dynamic_test "eigen_dynamic_test.cc")

add_executable(
        triangulate_test
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "triangulate_test.cc")

target_link_libraries(triangulate_test ${DEPEND_LIBS})

add_executable(sophus_log_test "sophus_log_test.cc")

target_link_libraries(sophus_log_test ${DEPEND_LIBS})

add_executable(notify_test "notify_test.cc")
target_link_libraries(notify_test pthread)

add_executable(expired_test "expired_test.cc")

add_executable(
        frontend_test
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/viewer.cc"
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
        "viewer_test.cc")
target_link_libraries(viewer_test ${DEPEND_LIBS})

add_executable(spdlog_test "spdlog_test.cc")

add_executable(se3d_cast_float "se3d_cast_float.cc")
target_link_libraries(se3d_cast_float ${DEPEND_LIBS})

add_executable(opengl_error_test "${PROJECT_SOURCE_DIR}/src/dataset.cc"
                                 "opengl_error_test.cc")
target_link_libraries(opengl_error_test ${DEPEND_LIBS})

add_executable(g2o_id_test "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
                           "g2o_id_test.cc")
target_link_libraries(g2o_id_test ${DEPEND_LIBS})

add_executable(
        backend_test
        "${PROJECT_SOURCE_DIR}/src/backend.cc"
        "${PROJECT_SOURCE_DIR}/src/camera.cc"
        "${PROJECT_SOURCE_DIR}/src/dataset.cc"
        "${PROJECT_SOURCE_DIR}/src/frontend.cc"
        "${PROJECT_SOURCE_DIR}/src/g2o_types.cc"
        "${PROJECT_SOURCE_DIR}/src/map.cc"
        "${PROJECT_SOURCE_DIR}/src/viewer.cc"
        "backend_test.cc")
target_link_libraries(backend_test ${DEPEND_LIBS})

add_executable(json_test "json_test.cc")
target_link_libraries(json_test ${DEPEND_LIBS})

add_executable(resize_logo "resize_logo.cc")
target_link_libraries(resize_logo ${DEPEND_LIBS})
