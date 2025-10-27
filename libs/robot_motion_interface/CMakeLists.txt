cmake_minimum_required(VERSION 3.15...3.26)
project(robot_motion_interface VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(TGT         ${PROJECT_NAME})                 # target name
set(EXPORT_NAME ${PROJECT_NAME}Targets)          # export set name
set(PKG         ${PROJECT_NAME})                 # package name
set(NS          ${PROJECT_NAME})                 # CMake namespace
set(PY_MODULE   robot_motion_interface_pybind)   # Python Binding Module


set(CPP_SOURCES
  cpp/src/panda_interface.cpp        
  cpp/src/tesollo/tesollo_dg3f_interface.cpp 
  cpp/src/tesollo/tesollo_communication.cpp  
)

set(CPP_LIBS
  robot_motion::robot_motion Franka::Franka pthread
)

# --------------- Shared Dependencies ---------------
find_package(robot_motion REQUIRED CONFIG)    
find_package(Franka 0.9 REQUIRED)  


# --------------- Python Binds ---------------
if( DEFINED SKBUILD)

  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(${PY_MODULE} MODULE
  cpp/src/bindings/python_bindings.cpp
  ${CPP_SOURCES}
  )

  target_include_directories(${PY_MODULE}
      PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
      $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${PY_MODULE} PRIVATE ${CPP_LIBS})

  install(TARGETS ${PY_MODULE} DESTINATION ${SKBUILD_PROJECT_NAME})

endif()



# ---------- System-wide C++ install (skip during pip wheel build) ----------
if(NOT DEFINED SKBUILD)


  add_library(${TGT} SHARED
      ${CPP_SOURCES}
  )

  target_include_directories(${TGT}
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
      $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${TGT} PUBLIC ${CPP_LIBS})
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


  # ---------- Examples ----------
  option(BUILD_EXAMPLES "Build example executables" ON)

  if(BUILD_EXAMPLES)
    add_executable(panda cpp/examples/panda.cpp)
    add_executable(tesollo cpp/examples/tesollo.cpp)

    target_link_libraries(panda PRIVATE ${TGT})
    target_link_libraries(tesollo PRIVATE ${TGT})
  endif()

endif()




