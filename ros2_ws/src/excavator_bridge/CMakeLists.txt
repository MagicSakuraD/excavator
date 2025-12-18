cmake_minimum_required(VERSION 3.8)
project(excavator_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# GStreamer via PkgConfig
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(include)

add_executable(ros_gst_bridge_node src/ros_gst_bridge.cpp)
ament_target_dependencies(ros_gst_bridge_node rclcpp sensor_msgs std_msgs)

# Link OpenCV
target_link_libraries(ros_gst_bridge_node ${OpenCV_LIBS})

# Link GStreamer
target_include_directories(ros_gst_bridge_node PUBLIC ${GST_INCLUDE_DIRS})
target_link_libraries(ros_gst_bridge_node ${GST_LIBRARIES})

install(TARGETS ros_gst_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()

