cmake_minimum_required(VERSION 3.5)
project(rc_joystick)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

include(FindPkgConfig)
pkg_check_modules(evdev REQUIRED libevdev)

include_directories(${evdev_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node rc_joystick_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  mavros_msgs
)
target_link_libraries(${PROJECT_NAME}_node ${evdev_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()