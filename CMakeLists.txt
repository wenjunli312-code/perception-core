cmake_minimum_required(VERSION 3.16)
project(perception-core VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(common-utils REQUIRED)
find_package(sensor-abstraction REQUIRED)
find_package(Eigen3 CONFIG)

add_library(perception_core STATIC
    src/detector.cpp
    src/point_pillars.cpp
    src/object_fusion.cpp
    src/lane_detector.cpp
)

target_include_directories(perception_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(perception_core
    PUBLIC
        common_utils::common_utils
        sensor_abstraction::sensor_abstraction
        $<$<TARGET_EXISTS:Eigen3::Eigen>:Eigen3::Eigen>
)

# Install
include(GNUInstallDirs)

install(TARGETS perception_core
    EXPORT perception-coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/perception_core
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT perception-coreTargets
    FILE perception-coreTargets.cmake
    NAMESPACE perception_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/perception-core
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/perception-coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/perception-coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/perception-core
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/perception-coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/perception-coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/perception-coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/perception-core
)

# Tests
enable_testing()
add_subdirectory(tests)
