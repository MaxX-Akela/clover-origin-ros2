cmake_minimum_required(VERSION 3.8)
project(aruco_pose)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Workaround for OpenCV 3/4 support
set(_opencv_version 4)
find_package(OpenCV ${_opencv_version} QUIET COMPONENTS core imgproc calib3d)
if (NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, searching for OpenCV 3")
  set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED COMPONENTS core imgproc calib3d)
if ("${OpenCV_VERSION_MINOR}" LESS "9")
  message(STATUS "OpenCV version too low, using vendored ArUco package")
  include(vendor/VendorOpenCV.cmake)
else()
  message(STATUS "Using system OpenCV ArUco package")
  find_package(OpenCV ${_opencv_version} REQUIRED COMPONENTS aruco)
endif()
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  "srv/SetMarkers.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
