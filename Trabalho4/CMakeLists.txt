cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# ---------- C++ executable nodes ----------
add_executable(localization_example_cpp src/localization_example.cpp src/localization_node.cpp)
ament_target_dependencies(localization_example_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs  
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS localization_example_cpp
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(mcl_localization_cpp src/mcl_localization.cpp src/localization_node.cpp)
ament_target_dependencies(mcl_localization_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS mcl_localization_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Python Node script ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/localization_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME localization_example_py
)

install(DIRECTORY launch scripts config
  DESTINATION share/${PROJECT_NAME}
)

# ---------- Testing ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
