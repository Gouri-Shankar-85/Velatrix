cmake_minimum_required(VERSION 3.8)
project(velatrix_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ******************************************
# Hexapod Config Library
# ******************************************

add_library(hexapod_config INTERFACE)

target_include_directories(hexapod_config INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Expor the library
ament_export_targets(hexapod_config)
ament_export_dependencies()

# Install the include folder
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library target
install(
  TARGETS hexapod_config
  EXPORT hexapod_config
  DESTINATION lib
)

# ******************************************
# Hexapod IK Library
# ******************************************

add_library(hexapod_ik
  src/hexapod_ik.cpp
)

target_include_directories(hexapod_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hexapod_ik
  hexapod_config   
)

# Export targets for ament
ament_export_targets(hexapod_ik)
ament_export_dependencies(rclcpp)  

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library target
install(
  TARGETS hexapod_ik
  EXPORT hexapod_ik
  DESTINATION lib
)

ament_package()
