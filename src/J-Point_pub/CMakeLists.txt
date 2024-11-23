cmake_minimum_required(VERSION 3.8)
project(J-Point_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(librealsense2_DIR "/opt/ros/humble/lib/x86_64-linux-gnu/cmake/realsense2/")




#find_package(librealsense2 REQUIRED PATHS "/opt/ros/humble/lib/x86_64-linux-gnu/cmake/realsense2/librealsense2Config.cmake")
find_package(librealsense2 REQUIRED)


#target_link_libraries(J-Point_pub PRIVATE realsense2)

add_executable(J-Point_pub src/sense_pub.cpp)
#target_link_libraries(J-Point_pub rclcpp sensor_msgs realsense2)
target_link_libraries(J-Point_pub realsense2)
ament_target_dependencies(J-Point_pub rclcpp std_msgs sensor_msgs librealsense2)
install(TARGETS
  J-Point_pub
  DESTINATION lib/${PROJECT_NAME})


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
