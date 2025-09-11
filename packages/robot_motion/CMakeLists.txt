cmake_minimum_required(VERSION 3.16)
project(robot_motion VERSION 0.1.0 LANGUAGES CXX)

# TODO use "${PROJECT_NAME}"

# Toggle static/shared with standard CMake switch
option(BUILD_SHARED_LIBS "Build shared libraries" ON)


add_library(robot_motion
    src/controllers/controller.cpp
    # src/controllers/joint_torque_controller.cpp
    # src/controllers/cartesian_torque_controller.cpp
    # src/ik/ik.cpp
    # src/ik/drake_ik.cpp
    src/robot_properties/robot_properties.cpp
    # src/robot_properties/panda_properties.cpp
)

# Public headers
target_include_directories(robot_motion
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Require C++17 
target_compile_features(robot_motion PUBLIC cxx_std_17)


# ------------- Link Libraries ------------------------
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN_INCLUDE_DIR})
target_link_libraries(robot_motion PUBLIC Eigen3::Eigen)





# ----- Installation & CMake package export -----
include(CMakePackageConfigHelpers)

install(TARGETS robot_motion
    EXPORT robot_motionTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install public headers
install(DIRECTORY include/ DESTINATION include)

# Generate and install CMake package config files
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/robot_motionConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfig.cmake"
    INSTALL_DESTINATION lib/cmake/robot_motion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfigVersion.cmake"
    DESTINATION lib/cmake/robot_motion
)

# Export targets so find_package works
install(EXPORT robot_motionTargets
    FILE robot_motionTargets.cmake
    NAMESPACE robot_motion::
    DESTINATION lib/cmake/robot_motion
)
