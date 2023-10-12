cmake_minimum_required(VERSION 3.10.2)
project(lingua_mechanica_kinematics_plugin)
  
if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(PkgConfig REQUIRED)
find_package(Torch REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(MSG_FILES
  RobotStates.msg
)

set(SRV_FILES
  GetPositoinsIK.srv
)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

set(LINGUA_MECHANICA_LIBRARY_NAME lingua_mechanica_kinematics_plugin)

add_library(${LINGUA_MECHANICA_LIBRARY_NAME} src/lingua_mechanica_kinematics_plugin.cpp)
target_link_libraries(${LINGUA_MECHANICA_LIBRARY_NAME} ${catkin_LIBRARIES} ${pkg_nlopt_LIBRARIES} ${TORCH_LIBRARIES})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ${LINGUA_MECHANICA_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  lingua_mechanica_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
