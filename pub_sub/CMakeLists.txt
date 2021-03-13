cmake_minimum_required(VERSION 2.8.3)
project(pub_sub)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   CustomMessage.msg
 )

add_service_files(
   FILES
   distance.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(subscriber src/sub.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})

add_executable(distance src/distance.cpp)
target_link_libraries(distance ${catkin_LIBRARIES})
add_dependencies(distance ${catkin_EXPORTED_TARGETS})

add_executable(publisher src/pub.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher ${catkin_EXPORTED_TARGETS})
#add_dependencies(pub_sub custom_messages_generate_messages_cpp)




