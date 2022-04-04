cmake_minimum_required(VERSION 3.0.2)
project(canbus_test)

add_compile_options(-std=c++20)

find_package(catkin REQUIRED COMPONENTS
  kacanopen)

catkin_package(
  CATKIN_DEPENDS kacanopen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(lely_DEPENDENCIES
  lely-coapp
  lely-util
  lely-io2
  lely-ev
)


add_executable(master_can src/master_can.cpp)
target_link_libraries (master_can
    ${catkin_LIBRARIES}
)

add_dependencies(master_can ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
