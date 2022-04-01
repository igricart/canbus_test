cmake_minimum_required(VERSION 3.0.2)
project(canbus_test)

add_compile_options(-std=c++20)

find_package(catkin REQUIRED COMPONENTS
  kacanopen)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES canbus_test
  CATKIN_DEPENDS kacanopen
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_slave src/slave.cpp)
target_link_libraries(${PROJECT_NAME}_slave
  lely-coapp
  lely-util
  lely-io2
  lely-ev
)

add_executable(${PROJECT_NAME}_master src/master.cpp)
target_link_libraries(${PROJECT_NAME}_master
  lely-coapp
  lely-util
  lely-io2
  lely-ev
)

add_executable(${PROJECT_NAME}_communicate src/communicate.cpp)

target_link_libraries (${PROJECT_NAME}_communicate
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_communicate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
