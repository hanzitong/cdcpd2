cmake_minimum_required(VERSION 3.9)
project(cdcpd)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CDCPD_TESTING OFF)

# https://blog.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(WARNING "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_utilities REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools QUIET)  # Changed to QUIET - not strictly required for core functionality
find_package(sdf_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(smmap_models QUIET)  # Temporarily optional
find_package(smmap_utilities QUIET)  # Temporarily optional
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(GUROBI QUIET)  # Disabled - using alternative optimization library
find_package(FAISS QUIET)  # Will use local build if not found in system
# Manually set FAISS paths if not found by find_package
if(NOT FAISS_FOUND)
  set(FAISS_INCLUDE_DIRS "/home/nachi/ws_cdcpd2/src/faiss" "/home/nachi/ws_cdcpd2/src/faiss/build")
  set(FAISS_LIBRARY "/home/nachi/ws_cdcpd2/src/faiss/build/faiss/libfaiss.a")
  if(EXISTS "${FAISS_LIBRARY}")
    set(FAISS_FOUND TRUE)
    message(STATUS "Using local FAISS build: ${FAISS_LIBRARY}")
  endif()
endif()

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Qt5 QUIET COMPONENTS Qt5 Core Xml Script OpenGL Svg)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters visualization features kdtree)

# horrid hack to remove the space at the beginning of the list
if(PCL_DEFINITIONS)
  list(LENGTH PCL_DEFINITIONS _len)
  if(_len GREATER 0)
    list(REMOVE_AT PCL_DEFINITIONS 0)
  endif()
endif()

### The library itself
add_library(cdcpd SHARED
        src/cdcpd.cpp
        # src/optimizer.cpp  # Disabled - requires GUROBI, will implement alternative
        src/obs_util.cpp
        src/past_template_matcher.cpp  # Re-enabled with FAISS
        )
target_include_directories(cdcpd PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(cdcpd SYSTEM PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})

if(FAISS_FOUND)
  target_include_directories(cdcpd SYSTEM PUBLIC ${FAISS_INCLUDE_DIRS})
  target_compile_definitions(cdcpd PUBLIC USE_FAISS)
  message(STATUS "FAISS enabled for cdcpd")
endif()

# Manually link ROS 2 dependencies
ament_target_dependencies(cdcpd
        rclcpp
        arm_utilities
        geometric_shapes
        geometry_msgs
        image_geometry
        image_transport
        cv_bridge
        pcl_conversions
        pcl_ros
        tf2_ros
        moveit_core
        moveit_msgs
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
        sdf_tools
        sensor_msgs
        #smmap_models  # Temporarily disabled
        #smmap_utilities  # Temporarily disabled
        std_msgs
        visualization_msgs
        )

target_link_libraries(cdcpd
        lapack
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${OpenCV_LIBS}
        CGAL::CGAL
        )

if(FAISS_FOUND)
  target_link_libraries(cdcpd ${FAISS_LIBRARY})
  message(STATUS "Linking FAISS library: ${FAISS_LIBRARY}")
endif()

# Gurobi disabled - will implement alternative optimization
# if(GUROBI_FOUND)
#   target_link_libraries(cdcpd PUBLIC ${GUROBI_LIBRARIES})
#   target_include_directories(cdcpd SYSTEM PUBLIC ${GUROBI_INCLUDE_DIRS})
#   target_compile_definitions(cdcpd PUBLIC USE_GUROBI)
# endif()
message(STATUS "GUROBI LIB ${GUROBI_LIBRARIES}")
message(STATUS "GUROBI INCLUDE ${GUROBI_INCLUDE_DIRS}")

target_compile_definitions(cdcpd PUBLIC ${PCL_DEFINITIONS})

# Set to private to not force downstream libraries to link against OpenMP
# https://cliutils.gitlab.io/modern-cmake/chapters/packages/OpenMP.html
# https://stackoverflow.com/a/51448364
find_package(OpenMP REQUIRED)
target_link_libraries(cdcpd OpenMP::OpenMP_CXX)

add_library(cdcpd::cdcpd ALIAS cdcpd)

# if (pybind11_catkin_FOUND)
#     pybind_add_module(pycdcpd src/bindings.cpp)
#     target_link_libraries(pycdcpd PUBLIC cdcpd)
# endif()

if (CDCPD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  message("Building tests!")
  add_subdirectory(tests)
endif()

### Installation
install(TARGETS cdcpd
        EXPORT cdcpd-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(FILES
        include/cdcpd/cdcpd.h
        include/cdcpd/optimizer.h
        include/cdcpd/obs_util.h
        include/cdcpd/past_template_matcher.h
        DESTINATION include/cdcpd)

install(EXPORT cdcpd-targets
        FILE cdcpd-targets.cmake
        NAMESPACE cdcpd::
        DESTINATION share/${PROJECT_NAME}/cmake)

add_executable(cdcpd_node
        src/cdcpd_ros/kinect_sub.cpp
        src/cdcpd_node.cpp
        )
ament_target_dependencies(cdcpd_node
        rclcpp
        sensor_msgs
        geometry_msgs
        std_msgs
        pcl_conversions
        tf2_ros
        visualization_msgs
        cv_bridge
        image_transport
        )
target_link_libraries(cdcpd_node cdcpd)

add_executable(cdcpd_offline
        src/cdcpd_ros/kinect_sub.cpp
        src/cdcpd_offline.cpp
        )
ament_target_dependencies(cdcpd_offline
        rclcpp
        sensor_msgs
        geometry_msgs
        std_msgs
        pcl_conversions
        tf2_ros
        visualization_msgs
        cv_bridge
        image_transport
        )
target_link_libraries(cdcpd_offline cdcpd)

install(TARGETS
        cdcpd_node
        cdcpd_offline
        DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_targets(cdcpd-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        arm_utilities
        geometric_shapes
        geometry_msgs
        image_geometry
        image_transport
        cv_bridge
        pcl_conversions
        pcl_ros
        tf2_ros
        moveit_core
        moveit_msgs
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
        sdf_tools
        sensor_msgs
        #smmap_models  # Temporarily disabled
        #smmap_utilities  # Temporarily disabled
        std_msgs
        visualization_msgs
        OpenCV
        Eigen3
        GUROBI
        FAISS
        Qt5
        PCL
        )

ament_package()
