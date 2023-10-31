cmake_minimum_required(VERSION 3.8)
project(canopen_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


generate_dcf(simple)
generate_dcf(canopen_system)
cogen_dcf(cia402_system)
generate_dcf(cia402)
generate_dcf(cia402_lifecycle)
cogen_dcf(cia402_diagnostics)
generate_dcf(simple_lifecycle)
cogen_dcf(simple_diagnostics)
generate_dcf(robot_control)
generate_dcf(eirabot_control)

install(DIRECTORY
  launch rviz urdf launch_tests meshes maps
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  if(CANOPEN_ENABLED)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(launch_tests/test_proxy_driver.py)
    add_launch_test(launch_tests/test_proxy_lifecycle_driver.py)
  endif()
endif()

ament_package()
