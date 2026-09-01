cmake_minimum_required(VERSION 3.8)
project(diff)

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ✅ Use transport_drivers' asio module
find_package(asio_cmake_module REQUIRED)
find_package(io_context REQUIRED)
find_package(serial_driver REQUIRED)  # ✅ For serial fallback (optional)

# -----------------------------
# Build the hardware plugin library
# -----------------------------
add_library(diffbot_system SHARED
  src/diffbot_system.cpp
)

# Include directories
target_include_directories(diffbot_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(diffbot_system
  rclcpp
  hardware_interface
  pluginlib
  io_context      # ✅ For async TCP
  serial_driver   # ✅ Optional: for serial fallback
)

# Optional: prevent name mangling
target_compile_definitions(diffbot_system PRIVATE "DIFFBOT_SYSTEM_BUILDING_DLL")

# -----------------------------
# Install
# -----------------------------
install(TARGETS diffbot_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES diffbot_system_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface diffbot_system_plugin.xml)

ament_package()