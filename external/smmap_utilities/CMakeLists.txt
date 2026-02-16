cmake_minimum_required(VERSION 3.9)
project(smmap_utilities)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Werror=return-type -Werror=missing-field-initializers)
endif()

## Add our local cmake modules to the list that can be found
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(arm_utilities REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kinematics_toolbox REQUIRED)
find_package(deformable_manipulation_experiment_params REQUIRED)
find_package(deformable_manipulation_msgs REQUIRED)
find_package(sdf_tools REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GUROBI)
find_package(NOMAD)

#############
## Library ##
#############
set(SRC
        src/grippers.cpp
        src/neighbours.cpp
        src/visualization_tools.cpp
        src/multiarm_bandits.cpp
        src/ros_communication_helpers.cpp
        src/trajectory.cpp)
add_library(${PROJECT_NAME} ${SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
        arm_utilities
        geometry_msgs
        kinematics_toolbox
        deformable_manipulation_experiment_params
        deformable_manipulation_msgs
        sdf_tools
        )
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

if (NOMAD_FOUND)
    add_library(${PROJECT_NAME}_nomad_solvers src/nomad_solvers.cpp)
    target_include_directories(${PROJECT_NAME}_nomad_solvers PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${NOMAD_INCLUDE_DIRS})
    ament_target_dependencies(${PROJECT_NAME}_nomad_solvers
            arm_utilities
            geometry_msgs
            kinematics_toolbox
            deformable_manipulation_experiment_params
            deformable_manipulation_msgs
            sdf_tools
            )
    target_link_libraries(${PROJECT_NAME}_nomad_solvers Eigen3::Eigen ${NOMAD_LIBRARIES})
endif ()

if (GUROBI_FOUND)
    add_library(${PROJECT_NAME}_gurobi_solvers src/gurobi_solvers.cpp)
    target_include_directories(${PROJECT_NAME}_gurobi_solvers PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${GUROBI_INCLUDE_DIRS})
    ament_target_dependencies(${PROJECT_NAME}_gurobi_solvers
            arm_utilities
            geometry_msgs
            kinematics_toolbox
            deformable_manipulation_experiment_params
            deformable_manipulation_msgs
            sdf_tools
            )
    target_link_libraries(${PROJECT_NAME}_gurobi_solvers Eigen3::Eigen ${GUROBI_LIBRARIES})
endif ()

## Installation
set(EXPORT_TARGETS ${PROJECT_NAME})
if (GUROBI_FOUND)
    list(APPEND EXPORT_TARGETS ${PROJECT_NAME}_gurobi_solvers)
endif ()
if (NOMAD_FOUND)
    list(APPEND EXPORT_TARGETS ${PROJECT_NAME}_nomad_solvers)
endif ()

install(TARGETS ${EXPORT_TARGETS}
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
        geometry_msgs
        kinematics_toolbox
        deformable_manipulation_experiment_params
        deformable_manipulation_msgs
        sdf_tools
        Eigen3
        )

if (GUROBI_FOUND)
    ament_export_dependencies(GUROBI)
endif()
if (NOMAD_FOUND)
    ament_export_dependencies(NOMAD)
endif()

ament_package()
