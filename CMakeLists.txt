cmake_minimum_required(VERSION 3.8)
project(data_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(slam_tracepoint_provider REQUIRED)

set(${PROJECT_NAME}_src 
  src/data_publisher.cpp
  src/euroc_publisher.cpp
)
add_library(${PROJECT_NAME}  SHARED ${${PROJECT_NAME}_src})
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src/
)

ament_target_dependencies(${PROJECT_NAME} 
"Boost" 
"cv_bridge"
"rclcpp" 
"rclcpp_components" 
"image_transport"
"OpenCV" 
"sensor_msgs" 
"slam_tracepoint_provider"
)

rclcpp_components_register_nodes(${PROJECT_NAME} "simulation::EurocPublisher")

add_executable(data_publisher_node src/data_publisher_node.cpp)
target_link_libraries(data_publisher_node
PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    ${PROJECT_NAME}
    data_publisher_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
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
