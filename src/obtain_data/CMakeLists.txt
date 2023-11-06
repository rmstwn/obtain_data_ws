cmake_minimum_required(VERSION 3.8)
project(obtain_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  Eigen3
  eigen3_cmake_module
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(obtain src/obtain.cpp)
ament_target_dependencies(obtain PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(publish src/publish.cpp)
ament_target_dependencies(publish PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
#rclcpp_components_register_nodes(jointstate "obtain_data::DynamixelStatePublisher")

# add_executable(dynamics src/dynamics.cpp)
# ament_target_dependencies(dynamics PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  obtain
  publish
  # dynamics
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
