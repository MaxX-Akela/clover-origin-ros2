cmake_minimum_required(VERSION 3.8)
project(roswww_static)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/update
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_update test/test_update.py TIMEOUT 60)
endif()

ament_package()
