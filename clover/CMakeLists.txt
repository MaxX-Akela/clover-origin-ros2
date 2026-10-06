cmake_minimum_required(VERSION 3.8)
project(clover)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# https://github.com/mavlink/mavros/blob/7f1a8/mavros/CMakeLists.txt#L42
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "srv/GetTelemetry.srv"
  "srv/Navigate.srv"
  "srv/NavigateGlobal.srv"
  "srv/SetAltitude.srv"
  "srv/SetYaw.srv"
  "srv/SetYawRate.srv"
  "srv/SetPosition.srv"
  "srv/SetVelocity.srv"
  "srv/SetAttitude.srv"
  "srv/SetRates.srv"
  "srv/SetLEDEffect.srv"
  "srv/Execute.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(simple_offboard src/simple_offboard.cpp)
target_include_directories(simple_offboard PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(simple_offboard
  ${cpp_typesupport_target}
  ${GeographicLib_LIBRARIES}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
)

add_executable(shell src/shell.cpp)
target_compile_options(shell PRIVATE -Wall -Wextra)
target_link_libraries(shell
  ${cpp_typesupport_target}
  rclcpp::rclcpp
)

add_executable(camera_markers src/camera_markers.cpp)
target_compile_options(camera_markers PRIVATE -Wall -Wextra)
target_link_libraries(camera_markers
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

add_executable(vpe_publisher src/vpe_publisher.cpp)
target_compile_options(vpe_publisher PRIVATE -Wall -Wextra)
target_link_libraries(vpe_publisher
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
)

add_executable(rc src/rc.cpp)
target_compile_options(rc PRIVATE -Wall -Wextra)
target_link_libraries(rc
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
)

# optical_flow is a component, optical_flow executable runs it in a separate process
add_library(optical_flow_component SHARED src/optical_flow.cpp)
target_compile_options(optical_flow_component PRIVATE -Wall -Wextra)
target_link_libraries(optical_flow_component
  rclcpp::rclcpp
  rclcpp_components::component
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
)
rclcpp_components_register_node(optical_flow_component
  PLUGIN "clover::OpticalFlow"
  EXECUTABLE optical_flow
)

install(TARGETS simple_offboard shell camera_markers vpe_publisher rc
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS optical_flow_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# rosidl generates the clover Python package itself, so the module can't be
# installed with ament_python_install_package; replace the generated __init__.py
install(FILES src/clover/__init__.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
