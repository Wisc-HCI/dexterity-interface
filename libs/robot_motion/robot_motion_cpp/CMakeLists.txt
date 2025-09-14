cmake_minimum_required(VERSION 3.16)
project(robot_motion VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

add_library(robot_motion
    src/controllers/controller.cpp
    src/robot_properties/robot_properties.cpp
)

target_include_directories(robot_motion
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)

target_compile_features(robot_motion PUBLIC cxx_std_17)

find_package(Eigen3 3.3 REQUIRED CONFIG)
target_link_libraries(robot_motion PUBLIC Eigen3::Eigen)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS robot_motion
    EXPORT robot_motionTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/robot_motionConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_motion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/robot_motionConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_motion
)

install(EXPORT robot_motionTargets
  FILE robot_motionTargets.cmake
  NAMESPACE robot_motion::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_motion
)
