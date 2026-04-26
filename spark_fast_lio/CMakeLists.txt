cmake_minimum_required(VERSION 3.20)
project(spark_fast_lio)

# Build type configuration - Release by default (no debug symbols)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread")
set(CMAKE_CXX_FLAGS "-O3")

# Debug build options
option(BUILD_DEBUG "Build with debug symbols and no optimization" OFF)
option(BUILD_RELWITHDEBINFO "Build with optimization and debug symbols" OFF)

if(BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
  set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -DDEBUG")
  message(STATUS "Building in DEBUG mode with full debug symbols")
elseif(BUILD_RELWITHDEBINFO)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")
  message(STATUS "Building in RelWithDebInfo mode with debug symbols")
else()
  message(STATUS "Building in Release mode without debug symbols")
  message(STATUS "To enable debug symbols: colcon build --cmake-args -DBUILD_DEBUG=ON")
  message(STATUS "Or: colcon build --cmake-args -DBUILD_RELWITHDEBINFO=ON")
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -fexceptions")

message(STATUS "Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message(STATUS "core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message(STATUS "core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(PCL_FIND_QUIETLY ON)
set(CLEAR_WARNING_SUPPRESSION OFF)
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CLEAR_WARNING_SUPPRESSION ON)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

if (CLEAR_WARNING_SUPPRESSION)
  unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
endif()

include_directories(
  include
  third_party/ikd-Tree
  third_party/IKFoM_toolkit)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#### for ikd-Tree
add_library(ikd_tree third_party/ikd-Tree/ikd_Tree.cpp)
# To resolve "recompile with -fPIC" issue
set_target_properties(ikd_tree PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- Fast-LIO2 algorithm core library (no ROS dependency) ----
add_library(fast_lio2_core SHARED
  src/fast_lio2_core.cpp
)
target_include_directories(fast_lio2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  third_party/ikd-Tree
  third_party/IKFoM_toolkit
)
target_link_libraries(fast_lio2_core
  ${PCL_LIBRARIES}
  ikd_tree
)
ament_target_dependencies(fast_lio2_core
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)
set_target_properties(fast_lio2_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Optional: Livox driver
find_package(livox_ros_driver QUIET)

# Create the ROS node component library
add_library(spark_lio_component SHARED
  src/spark_fast_lio.cpp
  src/preprocess.cpp
)

# Example: C++ standard
target_compile_features(spark_lio_component PUBLIC cxx_std_20)

# Link external libs
target_link_libraries(spark_lio_component
  ${PCL_LIBRARIES}
  ikd_tree
  fast_lio2_core
)

# If Livox found, link it here
if(livox_ros_driver_FOUND)
  target_link_libraries(spark_lio_component
    ${livox_ros_driver_LIBRARIES}
  )
  target_include_directories(spark_lio_component PRIVATE
    ${livox_ros_driver_INCLUDE_DIRS}
  )
  target_compile_definitions(spark_lio_component PRIVATE
    -DLIVOX_ROS_DRIVER_FOUND=${livox_ros_driver_FOUND}
  )
else()
  message(STATUS "Missing livox driver! AVIA Lidar will be disabled!")
endif()

# Pull in ament dependencies
ament_target_dependencies(
  spark_lio_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

rclcpp_components_register_node(
  spark_lio_component
  PLUGIN "spark_fast_lio::SPARKFastLIO2"
  EXECUTABLE spark_lio_mapping
)

# ---- SDK library (offline bag reader + syncPackages) ----
add_library(spark_lio_sdk SHARED
  src/spark_lio_sdk.cpp
  src/preprocess.cpp
)
target_include_directories(spark_lio_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(spark_lio_sdk
  ${PCL_LIBRARIES}
)
ament_target_dependencies(spark_lio_sdk
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
)

# ---- SDK demo executable ----
add_executable(spark_lio_sdk_demo
  src/parser_sdk.cpp
)
target_link_libraries(spark_lio_sdk_demo
  spark_lio_sdk
  ${PCL_LIBRARIES}
)
ament_target_dependencies(spark_lio_sdk_demo
  rclcpp
  sensor_msgs
)

# ---- SDK runner executable (Fast-LIO2 runner) ----
add_executable(spark_lio_run_sdk
  src/run_sdk.cpp
)
target_link_libraries(spark_lio_run_sdk
  spark_lio_sdk
  fast_lio2_core
  ${PCL_LIBRARIES}
)
ament_target_dependencies(spark_lio_run_sdk
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
)

# Install
install(
  TARGETS fast_lio2_core spark_lio_component spark_lio_sdk spark_lio_sdk_demo spark_lio_run_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Example for installing launch files, etc.
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Generate package info
ament_package()
