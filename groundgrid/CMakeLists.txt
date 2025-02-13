cmake_minimum_required(VERSION 3.14)
project(groundgrid)

# -------- default to C++17 --------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ----- clang compile options -----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----- find dependencies -----
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ----- link targets -----
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/groundgrid_node.cpp
  src/groundgrid.cpp
  src/config.cpp
  src/segmentation.cpp)

# ----- register composable nodes -----
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "groundgrid::GroundGridNode"
  EXECUTABLE groundgrid_node
)

# ----- for testing -----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch rviz)
