cmake_minimum_required(VERSION 3.8)
project(collarobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  "action/Gesture.action"
  "action/MoveIngredient.action"
)

ament_package()
