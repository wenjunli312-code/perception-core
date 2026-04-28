find_package(GTest QUIET)

if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
    )
    FetchContent_MakeAvailable(googletest)
endif()

add_executable(perception_core_tests
    test_detector.cpp
    test_point_pillars.cpp
    test_object_fusion.cpp
    test_lane_detector.cpp
)

target_link_libraries(perception_core_tests
    PRIVATE
        perception_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(perception_core_tests)
