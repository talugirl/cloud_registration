cmake_minimum_required(VERSION 2.8.3)
project(cloud_registration)

add_compile_options(-std=c++11)

Set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

find_package(catkin REQUIRED COMPONENTS
  
)
find_package(PCL REQUIRED )



catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS 
)

include_directories(
  src/
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(test_pcd
  src/test_pcd.cpp 
  )
target_link_libraries(test_pcd 
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  )
install(TARGETS test_pcd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)








