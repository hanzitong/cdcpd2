cmake_minimum_required(VERSION 3.9)
project(deformable_manipulation_experiment_params)

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
find_package(arm_utilities REQUIRED)
find_package(rclcpp REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

## Library file for the whole project
FILE(GLOB hpp_SRC "include/${PROJECT_NAME}/*.hpp")
FILE(GLOB h_SRC "include/${PROJECT_NAME}/*.h")
FILE(GLOB cpp_SRC "src/*.cpp")

add_library(${PROJECT_NAME} ${cpp_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
    arm_utilities
    rclcpp
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
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
    arm_utilities
    rclcpp
    Eigen3
)

ament_package()
