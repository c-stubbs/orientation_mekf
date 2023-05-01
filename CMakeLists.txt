cmake_minimum_required(VERSION 3.5)
project(mekf)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIR}
# )
include_directories(include)

add_library(${PROJECT_NAME}
  src/mekf.cpp
  )

add_executable(orientation_node src/orientation.cpp)
target_link_libraries(orientation_node ${PROJECT_NAME})
ament_target_dependencies(orientation_node rclcpp sensor_msgs std_msgs geometry_msgs tf2 tf2_ros)


install(TARGETS
  orientation_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()