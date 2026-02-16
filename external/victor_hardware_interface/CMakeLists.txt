cmake_minimum_required(VERSION 3.9)
project(victor_hardware_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CartesianControlModeLimits.msg"
    "msg/CartesianImpedanceParameters.msg"
    "msg/CartesianPathExecutionParameters.msg"
    "msg/CartesianValueQuantity.msg"
    "msg/ControlMode.msg"
    "msg/ControlModeParameters.msg"
    "msg/JointImpedanceParameters.msg"
    "msg/JointPathExecutionParameters.msg"
    "msg/JointValueQuantity.msg"
    "msg/MotionCommand.msg"
    "msg/MotionStatus.msg"
    "msg/Robotiq3FingerActuatorCommand.msg"
    "msg/Robotiq3FingerActuatorStatus.msg"
    "msg/Robotiq3FingerCommand.msg"
    "msg/Robotiq3FingerObjectStatus.msg"
    "msg/Robotiq3FingerStatus.msg"
    "msg/Robotiq3FingerStatusSync.msg"
    "msg/GraspStatus.msg"
    DEPENDENCIES
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

# Export dependencies
ament_export_dependencies(
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    rosidl_default_runtime
)

ament_package()

