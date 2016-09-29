cmake_minimum_required(VERSION 2.8.3)

project(cs_merge_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs nav_msgs)

add_message_files(FILES
  transform.msg
)

add_service_files(FILES
  getTransform.srv getWorld.srv
)

generate_messages(DEPENDENCIES std_msgs nav_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs nav_msgs
)
