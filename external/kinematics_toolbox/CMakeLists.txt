cmake_minimum_required(VERSION 3.9)
project(kinematics_toolbox)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

#######################
## Build - Libraries ##
#######################

add_library(${PROJECT_NAME}
  src/kinematics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_library(${PROJECT_NAME}_ros_types
  src/kinematics_ros_types.cpp
)
target_include_directories(${PROJECT_NAME}_ros_types PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_ros_types
  geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_ros_types Eigen3::Eigen)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros_types
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION include)

install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

# Export dependencies
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    geometry_msgs
    Eigen3
)

ament_package()
