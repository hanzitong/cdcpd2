cmake_minimum_required(VERSION 3.9)
project(sdf_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

option(SDF_TOOLS_PYTHON_BINDINGS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(arm_utilities REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
# MoveIt! isn't a required dependency
find_package(moveit_msgs REQUIRED)
find_package(moveit_core QUIET)
find_package(moveit_ros_planning QUIET)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SDF.msg"
    "msg/CollisionMap.msg"
    "msg/TaggedObjectCollisionMap.msg"
    "srv/ComputeSDF.srv"
    DEPENDENCIES geometry_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
if(moveit_core_FOUND)
    message(STATUS "MoveIt! found. Building SDF Tools library with MoveIt! integration.")
    # SDF library
    add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/collision_map.cpp
        src/${PROJECT_NAME}/tagged_object_collision_map.cpp
        src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
        src/${PROJECT_NAME}/sdf.cpp
        src/${PROJECT_NAME}/sdf_builder.cpp)
    
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        arm_utilities
        moveit_core
        moveit_ros_planning
        moveit_msgs
        )
    
    target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        ${OpenCV_LIBRARIES})
    
    # This is required to access the generated message headers
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
    
    # SDF generation node
    add_executable(sdf_generation_node src/sdf_generation_node.cpp)
    ament_target_dependencies(sdf_generation_node
        rclcpp
        moveit_core
        moveit_msgs
        moveit_ros_planning
        )
    target_link_libraries(sdf_generation_node ${PROJECT_NAME})
    
else()
    message(STATUS "MoveIt! not found. Building SDF Tools library without MoveIt! integration.")
    # SDF library
    add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/collision_map.cpp
        src/${PROJECT_NAME}/tagged_object_collision_map.cpp
        src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
        src/${PROJECT_NAME}/sdf.cpp)
    
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        arm_utilities
        )
    
    target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        ${OpenCV_LIBRARIES})
    
    # This is required to access the generated message headers
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
endif()

# Simple test nodes for voxel_grid
add_executable(voxel_grid_test src/test_voxel_grid.cpp)
ament_target_dependencies(voxel_grid_test rclcpp)
target_link_libraries(voxel_grid_test ${PROJECT_NAME})

add_executable(estimate_distance_test src/estimate_distance_test.cpp)
ament_target_dependencies(estimate_distance_test rclcpp)
target_link_libraries(estimate_distance_test ${PROJECT_NAME})

add_executable(compute_convex_segments_test src/compute_convex_segments_test.cpp)
ament_target_dependencies(compute_convex_segments_test rclcpp)
target_link_libraries(compute_convex_segments_test ${PROJECT_NAME})

# Image to SDF node
add_executable(image_2d_sdf_node src/image_2d_sdf_node.cpp)
ament_target_dependencies(image_2d_sdf_node rclcpp cv_bridge)
target_link_libraries(image_2d_sdf_node ${OpenCV_LIBRARIES})

# Tutorial on how to use the components of the sdf_tools library
add_executable(sdf_tools_tutorial src/sdf_tools_tutorial.cpp)
ament_target_dependencies(sdf_tools_tutorial rclcpp)
target_link_libraries(sdf_tools_tutorial ${PROJECT_NAME})

#####################
## Python Bindings ##
#####################

# Note: Python bindings would need to be reimplemented with pybind11 directly
# or using other ROS2-compatible Python binding methods
# Commenting out for now
# if (${SDF_TOOLS_PYTHON_BINDINGS})
#     find_package(pybind11 REQUIRED)
#     pybind11_add_module(pysdf_tools src/sdf_tools/bindings.cpp)
#     target_link_libraries(pysdf_tools sdf_tools)
# endif()

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS
    voxel_grid_test
    estimate_distance_test
    compute_convex_segments_test
    image_2d_sdf_node
    sdf_tools_tutorial
    DESTINATION lib/${PROJECT_NAME})

if(moveit_core_FOUND)
    install(TARGETS sdf_generation_node
        DESTINATION lib/${PROJECT_NAME})
endif()

## Mark library for installation
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    PATTERN ".svn" EXCLUDE)

install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

# Export dependencies
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    image_transport
    cv_bridge
    arm_utilities
    moveit_msgs
    Eigen3
    OpenCV
    rosidl_default_runtime
    )

if(moveit_core_FOUND)
    ament_export_dependencies(moveit_core moveit_ros_planning)
endif()

ament_package()
