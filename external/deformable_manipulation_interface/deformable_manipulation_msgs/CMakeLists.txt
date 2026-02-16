cmake_minimum_required(VERSION 3.9)
project(deformable_manipulation_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sdf_tools REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages, services and actions
rosidl_generate_interfaces(${PROJECT_NAME}
    # Messages
    "msg/ConfidenceStamped.msg"
    "msg/VectorPose.msg"
    "msg/VectorConfiguration.msg"
    "msg/WorldState.msg"
    "msg/RobotPathTest.msg"
    "msg/RobotPathTestResult.msg"
    "msg/TransitionTest.msg"
    "msg/TransitionTestResult.msg"
    # Services
    "srv/ExecuteRobotMotion.srv"
    "srv/GetGripperNames.srv"
    "srv/GetGripperAttachedNodeIndices.srv"
    "srv/GetGripperCollisionReport.srv"
    "srv/GetGripperPose.srv"
    "srv/GetGripperStretchingVectorInfo.srv"
    "srv/GetPointSet.srv"
    "srv/GetVector3Set.srv"
    "srv/GetPoseSet.srv"
    "srv/GetMirrorLine.srv"
    "srv/GetFreeSpaceGraph.srv"
    "srv/GetSignedDistanceField.srv"
    "srv/GetRobotConfiguration.srv"
    "srv/TestRobotMotionMicrosteps.srv"
    "srv/TransitionTestingVisualization.srv"
    # Actions
    "action/TestRobotMotion.action"
    "action/GenerateTransitionData.action"
    "action/TestRobotPaths.action"
    DEPENDENCIES
    action_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    sdf_tools
)

###########
## Build ##
###########

## Install header files
install(DIRECTORY include/
    DESTINATION include)

# Export dependencies
ament_export_dependencies(
    action_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    sdf_tools
    rosidl_default_runtime
)

ament_package()
