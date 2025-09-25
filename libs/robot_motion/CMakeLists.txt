cmake_minimum_required(VERSION 3.16)
project(robot_motion VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(TGT         ${PROJECT_NAME})                 # target name
set(EXPORT_NAME ${PROJECT_NAME}Targets)          # export set name
set(PKG         ${PROJECT_NAME})                 # package name
set(NS          ${PROJECT_NAME})                 # CMake namespace
set(PY_MODULE   robot_motion_pybind)       # Python Binding Module

# --------------- Dependencies ---------------
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

# Ask Python where pybind11’s CMake package lives and add it to search path
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
  OUTPUT_VARIABLE _pybind11_cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmake_dir}")
find_package(pybind11 CONFIG REQUIRED)

find_package(Eigen3 3.3 REQUIRED CONFIG)


# ---------- C++ library ----------
add_library(${TGT}
    cpp/src/controllers/controller.cpp
    cpp/src/controllers/joint_torque_controller.cpp
    cpp/src/robot_properties/robot_properties.cpp
)

target_include_directories(${TGT}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
      $<INSTALL_INTERFACE:include>
)

target_link_libraries(${TGT} PUBLIC Eigen3::Eigen)
target_compile_features(${TGT} PUBLIC cxx_std_17)


# --------------- Python Binds ---------------

pybind11_add_module(${PY_MODULE} cpp/src/bindings/python_bindings.cpp)
target_link_libraries(${PY_MODULE} PRIVATE ${TGT})
target_include_directories(${PY_MODULE} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
)

target_link_libraries(${PY_MODULE} PRIVATE robot_motion)

# Put the extension next to the pure-Python files in the built wheel:
# scikit-build-core sets SKBUILD_PLATLIB_DIR to the wheel's site-packages dir.
install(TARGETS ${PY_MODULE}
  LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/robot_motion
  RUNTIME DESTINATION ${SKBUILD_PLATLIB_DIR}/robot_motion
)




# ---------- System-wide C++ install (skip during pip wheel build) ----------

if(NOT DEFINED SKBUILD)

  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  install(TARGETS ${TGT}
      EXPORT ${EXPORT_NAME}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  install(DIRECTORY cpp/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PKG}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
  )

  configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PKG}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PKG}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PKG}
  )

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PKG}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PKG}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PKG}
  )

  install(EXPORT ${EXPORT_NAME}
    FILE ${EXPORT_NAME}.cmake
    NAMESPACE ${NS}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PKG}
  )

endif()
