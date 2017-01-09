cmake_minimum_required(VERSION 3.5)

project(gazebo_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_generation REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# message and service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContactsState.msg"
  "msg/ContactState.msg"
  "msg/LinkState.msg"
  "msg/LinkStates.msg"
  "msg/ModelState.msg"
  "msg/ModelStates.msg"
  "msg/ODEJointProperties.msg"
  "msg/ODEPhysics.msg"
  "msg/WorldState.msg"
   "srv/ApplyBodyWrench.srv"
   "srv/DeleteModel.srv"
   "srv/GetLinkState.srv"
   "srv/GetPhysicsProperties.srv"
   "srv/SetJointProperties.srv"
   "srv/SetModelConfiguration.srv"
   "srv/SpawnModel.srv"
   "srv/ApplyJointEffort.srv"
   "srv/GetJointProperties.srv"
   "srv/GetModelProperties.srv"
   "srv/GetWorldProperties.srv"
   "srv/SetLinkProperties.srv"
   "srv/SetModelState.srv"
   "srv/BodyRequest.srv"
   "srv/GetLinkProperties.srv"
   "srv/GetModelState.srv"
   "srv/JointRequest.srv"
   "srv/SetLinkState.srv"
   "srv/SetPhysicsProperties.srv"
   "srv/SetJointTrajectory.srv"
   DEPENDENCIES builtin_interfaces
)


ament_package()  
