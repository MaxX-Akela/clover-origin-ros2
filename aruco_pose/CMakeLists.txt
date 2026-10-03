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

add_library(${PROJECT_NAME}_components SHARED
  src/aruco_detect.cpp
  src/aruco_map.cpp
  src/draw.cpp
)
target_include_directories(${PROJECT_NAME}_components PRIVATE ${OpenCV_INCLUDE_DIRS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_components
  ${cpp_typesupport_target}
  ${OpenCV_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_components::component
  image_transport::image_transport
  cv_bridge::cv_bridge
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  message_filters::message_filters
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

rclcpp_components_register_nodes(${PROJECT_NAME}_components "aruco_pose::ArucoDetect")
rclcpp_components_register_nodes(${PROJECT_NAME}_components "aruco_pose::ArucoMap")

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS src/genmap.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
