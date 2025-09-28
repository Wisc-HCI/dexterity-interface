cmake_minimum_required(VERSION 3.15...3.26)
project(rm_2 LANGUAGES CXX)

set(TGT         ${PROJECT_NAME})                 # target name
set(EXPORT_NAME ${PROJECT_NAME}Targets)          # export set name
set(PKG         ${PROJECT_NAME})                 # package name
set(NS          ${PROJECT_NAME})                 # CMake namespace
set(PY_MODULE   rm_2_pybind)       # Python Binding Module

# --------------- Dependencies ---------------

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

find_package(Eigen3 3.3 REQUIRED CONFIG)



# ---------- C++ library ----------
# add_library(${TGT} SHARED
#     cpp/src/controllers/controller.cpp
#     cpp/src/controllers/joint_torque_controller.cpp
#     cpp/src/robot_properties/robot_properties.cpp
# )

# target_include_directories(${TGT}
#     PUBLIC
#       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
#       $<INSTALL_INTERFACE:include>
# )

# target_link_libraries(${TGT} PUBLIC Eigen3::Eigen)


# --------------- Python Binds ---------------
message(STATUS "SKBUILD_PROJECT_NAME = ${SKBUILD_PROJECT_NAME}")

# pybind11_add_module(${PY_MODULE} MODULE cpp/src/bindings/python_bindings.cpp)
pybind11_add_module(${PY_MODULE} MODULE
  cpp/src/bindings/python_bindings.cpp
  cpp/src/controllers/controller.cpp
  cpp/src/controllers/joint_torque_controller.cpp
  cpp/src/robot_properties/robot_properties.cpp
)

target_include_directories(${PY_MODULE}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
      $<INSTALL_INTERFACE:include>
)

# target_link_libraries(${PY_MODULE} PRIVATE  ${TGT} Eigen3::Eigen)
target_link_libraries(${PY_MODULE} PRIVATE  Eigen3::Eigen)

install(TARGETS ${PY_MODULE} DESTINATION ${SKBUILD_PROJECT_NAME})

