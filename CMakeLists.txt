cmake_minimum_required(VERSION 2.8.3)
project(cloud_registration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  
)
find_package(PCL REQUIRED )
find_package(Eigen REQUIRED)


catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  src/
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

add_executable(test_pcd
  src/test_pcd.cpp 
  )
target_link_libraries(test_pcd 
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )
install(TARGETS test_pcd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)








