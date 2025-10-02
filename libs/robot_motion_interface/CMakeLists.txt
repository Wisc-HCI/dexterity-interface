cmake_minimum_required(VERSION 3.16)
project(robot_motion_interface VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(TGT         ${PROJECT_NAME})                 # target name
set(EXPORT_NAME ${PROJECT_NAME}Targets)          # export set name
set(PKG         ${PROJECT_NAME})                 # package name
set(NS          ${PROJECT_NAME})                 # CMake namespace



add_library(${TGT})
target_sources(${TGT}
  PRIVATE
    cpp/src/panda_interface.cpp        
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/robot_motion_interface/interface.hpp>
)
target_include_directories(${TGT}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${TGT} PUBLIC cxx_std_17)

# ---------- Dependencies ---------------
find_package(robot_motion CONFIG REQUIRED)    
find_package(Franka 0.9 REQUIRED)  

target_link_libraries(${TGT}
  PUBLIC
    robot_motion::robot_motion
    Franka::Franka
)


# ---------- Install / package ----------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${TGT}
  EXPORT ${EXPORT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY cpp/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Config files
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
  target_link_libraries(panda PRIVATE ${TGT})
endif()

