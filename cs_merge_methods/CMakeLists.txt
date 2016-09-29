cmake_minimum_required(VERSION 2.8.3)

project(cs_merge_methods)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cs_merge_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  cs_merge_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cs_merge_icp_svd src/cs_merge_icp_svd.cpp)
add_dependencies(cs_merge_icp_svd ${catkin_EXPORTED_TARGETS})
target_link_libraries(cs_merge_icp_svd ${catkin_LIBRARIES})

add_executable(cs_merge_icp_gradient src/cs_merge_icp_gradient.cpp)
add_dependencies(cs_merge_icp_gradient ${catkin_EXPORTED_TARGETS})
target_link_libraries(cs_merge_icp_gradient ${catkin_LIBRARIES})

add_executable(cs_merge_hough_ccr src/cs_merge_hough_ccr.cpp)
add_dependencies(cs_merge_hough_ccr ${catkin_EXPORTED_TARGETS})
target_link_libraries(cs_merge_hough_ccr ${catkin_LIBRARIES})

add_executable(cs_merge_hough_corner src/cs_merge_hough_corner.cpp)
add_dependencies(cs_merge_hough_corner ${catkin_EXPORTED_TARGETS})
target_link_libraries(cs_merge_hough_corner ${catkin_LIBRARIES})


