cmake_minimum_required(VERSION 3.8)
project(image_enhancer_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
find_package(OpenCV REQUIRED)

add_executable(enhancer src/enhancer.cpp)
ament_target_dependencies(enhancer rclcpp sensor_msgs cv_bridge)
target_link_libraries(enhancer ${OpenCV_LIBS})

install(TARGETS enhancer
  DESTINATION lib/${PROJECT_NAME})
