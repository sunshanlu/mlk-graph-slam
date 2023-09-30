find_package(OpenCV 3.1 REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)

add_executable(createCameraTest createCameraTest.cpp)
target_link_libraries(
        createCameraTest
        fmt::fmt
)

add_executable(cvPyrLKTest cvPyrLKTest.cpp)
target_link_libraries(
        cvPyrLKTest
        ${OpenCV_LIBS}
)

add_executable(fmtTest fmtTest.cpp)
target_link_libraries(
        fmtTest
        fmt::fmt
)

add_executable(opencvTest opencvTest.cpp)
target_link_libraries(
        opencvTest
        ${OpenCV_LIBS}
)

add_executable(se3Default se3Default.cpp)
target_link_libraries(
        se3Default
        Sophus::Sophus
)

add_executable(vectorTest vectorTest.cpp)