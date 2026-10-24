cmake_minimum_required(VERSION 3.8)
project(robot_localization_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pf src/pf.cpp src/helper_functions.cpp src/occupancy_field.cpp src/angle_helpers.cpp)
ament_target_dependencies(pf rclcpp)
ament_target_dependencies(pf geometry_msgs)
ament_target_dependencies(pf sensor_msgs)
ament_target_dependencies(pf nav2_msgs)
ament_target_dependencies(pf tf2_ros)
ament_target_dependencies(pf tf2)

install(TARGETS
  pf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav_msgs REQUIRED)

  ament_add_gtest(test_pf
    test/test_pf.cpp
    src/pf.cpp
    src/helper_functions.cpp
    src/occupancy_field.cpp
    src/angle_helpers.cpp
  )

  if(TARGET test_pf)
    target_compile_definitions(test_pf
      PRIVATE PARTICLE_FILTER_UNIT_TEST
    )

    target_include_directories(test_pf PRIVATE src)

    ament_target_dependencies(test_pf
      rclcpp
      geometry_msgs
      sensor_msgs
      nav2_msgs
      nav_msgs
      tf2_ros
      tf2
    )

    target_link_libraries(test_pf Eigen3::Eigen)
  endif()

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
