cmake_minimum_required(VERSION 3.8)
project(startup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros2_control REQUIRED
  COMPONENTS
    controller_manager
    controller_interface
    hardware_interface
    transmission_interface
)

# Install urdf/, config/, and launch/ preserving their folder names under share/
# So you’ll get:
#   install/share/urdf/…
#   install/share/config/…
#   install/share/launch/…
# -----------------------------------------------------------------------------

# Add executable
add_executable(magnetometer_converter src/magnetometer_converter.cpp)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
ament_target_dependencies(magnetometer_converter 
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_geometry_msgs
)

# Install executable
install(TARGETS
  magnetometer_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.yml"
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
    PATTERN "*.launch.py"
    PATTERN "*.launch.xml"
)

# cmpile xacro -> urdf
file(GLOB XACRO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro")
set(GENERATED_URDFS "")
set(GENERATED_URDF_DIR "${CMAKE_CURRENT_BINARY_DIR}/urdf")
foreach(src IN LISTS XACRO_FILES)
  get_filename_component(name "${src}" NAME_WE)  # strip path & .xacro
  set(out "${GENERATED_URDF_DIR}/${name}.urdf")
  list(APPEND GENERATED_URDFS "${out}")

  add_custom_command(
    OUTPUT "${out}"
    COMMAND xacro "${src}" -o "${out}"
    DEPENDS "${src}"
    COMMENT "Generating URDF from ${name}.xacro"
  )
endforeach()
add_custom_target(
  generate_urdfs ALL
  DEPENDS ${GENERATED_URDFS}
)
install(
  DIRECTORY "${GENERATED_URDF_DIR}/"
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf"
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
