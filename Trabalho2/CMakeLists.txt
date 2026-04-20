cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)


include_directories(include)

# ---------- C++ executable node ----------
add_executable(planning_example_cpp src/planning_example.cpp src/planning_node.cpp)
ament_target_dependencies(planning_example_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS planning_example_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config scripts
        DESTINATION share/${PROJECT_NAME})

# ---------- Python Node script ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/planning_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME planning_example_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
