cmake_minimum_required(VERSION 2.8.3)
project(mujoco_benchmark)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "-Wall --std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Boost REQUIRED COMPONENTS program_options system)
find_package(Eigen REQUIRED)
find_package(MuJoCo REQUIRED)


include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS}
)
link_directories(
  ${Boost_LIBRARY_DIRS}
)

catkin_package()

add_executable("${PROJECT_NAME}" src/mujoco_benchmark.cpp)
target_link_libraries("${PROJECT_NAME}"
  ${Boost_LIBRARIES}
  ${MUJOCO_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS "${PROJECT_NAME}"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
