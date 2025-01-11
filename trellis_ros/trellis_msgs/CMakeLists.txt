cmake_minimum_required(VERSION 3.5)
project(trellis_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)

# Generate messages.
rosidl_generate_interfaces(${PROJECT_NAME}
    srv/Synthesize3DAsset.srv
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(
    rosidl_default_runtime
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()
endif()

ament_package()
