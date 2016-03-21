cmake_minimum_required(VERSION 2.8.3)
project(dart_benchmark)

find_package(catkin REQUIRED COMPONENTS aikido_rviz)
find_package(aikido REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system)
find_package(DART REQUIRED COMPONENTS core)

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${Boost_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "-Wall --std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package()

add_executable("${PROJECT_NAME}" src/dart_benchmark.cpp)
target_link_libraries("${PROJECT_NAME}"
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS "${PROJECT_NAME}"
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
