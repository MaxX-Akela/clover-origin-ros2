cmake_minimum_required(VERSION 3.8)
project(clover)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
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
find_package(led_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
target_link_libraries(shell
  ${cpp_typesupport_target}
  rclcpp::rclcpp
)

add_executable(camera_markers src/camera_markers.cpp)
target_link_libraries(camera_markers
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

add_executable(vpe_publisher src/vpe_publisher.cpp)
target_link_libraries(vpe_publisher
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
)

add_executable(rc src/rc.cpp)
target_link_libraries(rc
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
)

add_executable(led src/led.cpp)
target_link_libraries(led
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
  ${led_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
)

# optical_flow is a component, optical_flow executable runs it in a separate process
add_library(optical_flow_component SHARED src/optical_flow.cpp)
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

install(TARGETS simple_offboard shell camera_markers vpe_publisher rc led
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS optical_flow_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS src/selfcheck.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME selfcheck
)

install(PROGRAMS src/mavros_params.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mavros_params
)

install(PROGRAMS src/waitfile
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY config camera_info udev examples
  DESTINATION share/${PROJECT_NAME}
)

# Web pages, found by roswww_static in share/clover/www.
# clover.log, clover_version and docs are symlinks in the original repository, here they are placeholder
# files containing the link target, so the links are created on install; docs (documentation
# snapshot) is skipped.
install(DIRECTORY www
  DESTINATION share/${PROJECT_NAME}
  PATTERN "CATKIN_IGNORE" EXCLUDE
  PATTERN "clover.log" EXCLUDE
  PATTERN "clover_version" EXCLUDE
  PATTERN "docs" EXCLUDE
)
install(CODE "
  file(CREATE_LINK /var/log/clover.log \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/www/clover.log\" SYMBOLIC)
  file(CREATE_LINK /etc/clover_version \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/www/clover_version\" SYMBOLIC)
")

# rosidl generates the clover Python package itself, so the module can't be
# installed with ament_python_install_package; replace the generated __init__.py
install(FILES src/clover/__init__.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # the tests use the same ROS_DOMAIN_ID, so they should not be run in parallel
  foreach(_test basic offboard examples test_www test_shell test_camera_markers test_rc test_vpe_publisher test_led
      test_optical_flow test_selfcheck test_launch test_web)
    add_launch_test(test/${_test}.py TARGET clover_${_test} TIMEOUT 300)
    set_tests_properties(clover_${_test} PROPERTIES RUN_SERIAL TRUE)
  endforeach()
endif()

ament_package()
