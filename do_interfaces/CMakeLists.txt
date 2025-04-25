cmake_minimum_required(VERSION 3.8)
project(do_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Add the following 1 line if your message depends on type *time*
########################################
find_package(builtin_interfaces REQUIRED)
########################################

# Add the following lines to CMakeLists.txt
########################################
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DO.msg"
  # Add the following if your message depends on type *time*
  DEPENDENCIES builtin_interfaces
)

########################################

ament_export_dependencies(rosidl_default_runtime)

ament_package()
