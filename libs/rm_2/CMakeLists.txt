cmake_minimum_required(VERSION 3.15...3.26)
project(rm_2 VERSION 0.0.1 LANGUAGES CXX)

set(TGT         ${PROJECT_NAME})                 # target name
set(EXPORT_NAME ${PROJECT_NAME}Targets)          # export set name
set(PKG         ${PROJECT_NAME})                 # package name
set(NS          ${PROJECT_NAME})                 # CMake namespace
set(PY_MODULE   rm_2_pybind)       # Python Binding Module

set(CPP_SOURCES
  cpp/src/controllers/controller.cpp
  cpp/src/controllers/joint_torque_controller.cpp
  cpp/src/robot_properties/robot_properties.cpp
)

# --------------- Dependencies ---------------


find_package(Eigen3 3.3 REQUIRED CONFIG)




# --------------- Python Binds ---------------

if( DEFINED SKBUILD)

    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 CONFIG REQUIRED)

    pybind11_add_module(${PY_MODULE} MODULE
    cpp/src/bindings/python_bindings.cpp
    ${CPP_SOURCES}
    )

    target_include_directories(${PY_MODULE}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
        $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(${PY_MODULE} PRIVATE  Eigen3::Eigen)

    install(TARGETS ${PY_MODULE} DESTINATION ${SKBUILD_PROJECT_NAME})

endif()


# ---------- System-wide C++ install (skip during pip wheel build) ----------


if(NOT DEFINED SKBUILD)
    message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "CMAKE_INSTALL_LIBDIR = ${CMAKE_INSTALL_LIBDIR}")


    add_library(${TGT} SHARED
        ${CPP_SOURCES}
    )

    target_include_directories(${TGT}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
        $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(${TGT} PUBLIC Eigen3::Eigen)
    target_compile_features(${TGT} PUBLIC cxx_std_17)


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