cmake_minimum_required(VERSION 3.8)
project(velatrix_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------
# Helper function to add a library
# -----------------------------
function(add_hexapod_library lib_name src_file deps)
  add_library(${lib_name} src/${src_file})
  target_include_directories(${lib_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  if(deps)
    target_link_libraries(${lib_name} ${deps})
  endif()
  ament_export_targets(${lib_name})
  ament_export_dependencies(rclcpp)
  install(
    DIRECTORY include/
    DESTINATION include
  )
  install(
    TARGETS ${lib_name}
    EXPORT ${lib_name}
    DESTINATION lib
  )
endfunction()

# -----------------------------
# Hexapod libraries
# -----------------------------

# 1. hexapod_config (header-only)
add_library(hexapod_config INTERFACE)
target_include_directories(hexapod_config INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(hexapod_config)
ament_export_dependencies(rclcpp)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS hexapod_config
  EXPORT hexapod_config
  DESTINATION lib
)

# 2. hexapod_body 
add_hexapod_library(hexapod_body hexapod_body.cpp hexapod_config)

# 3. hexapod_ik
add_hexapod_library(hexapod_ik hexapod_ik.cpp "hexapod_config;hexapod_body")

# 4. hexapod_fk
add_hexapod_library(hexapod_fk hexapod_fk.cpp "hexapod_config;hexapod_body")

#5. hexapod_gait
add_hexapod_library(hexapod_gait hexapod_gait.cpp "hexapod_config;hexapod_body")

ament_package()
