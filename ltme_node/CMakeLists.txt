cmake_minimum_required(VERSION 3.5)
project(ltme_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ltme_interfaces REQUIRED)

set(LDCP_SDK_DIR "lib/ldcp_sdk")
add_subdirectory(${LDCP_SDK_DIR})

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${LDCP_SDK_DIR}/Sources/include"
)

add_executable(ltme_node
  "src/ltme_node.cpp"
)
# target_compile_options(ltme_node PRIVATE -Wno-pedantic)
ament_target_dependencies(ltme_node
  rclcpp
  sensor_msgs
  std_srvs
  ltme_interfaces
)
target_link_libraries(ltme_node
  ldcp_sdk
)

install(TARGETS ltme_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
