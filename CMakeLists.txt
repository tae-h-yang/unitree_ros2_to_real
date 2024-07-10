cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)

# For the nav2 to go1 cmd node.
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
message("-- Architecture Detected: ${ARCH}")
set(EXTRA_LIBS -pthread ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib/libunitree_legged_sdk_${ARCH}.so lcm)
message("-- EXTRA_LIBS: ${EXTRA_LIBS}")


set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/include
)



link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

# Add the new test.cpp file
add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${EXTRA_LIBS})
ament_target_dependencies(test_node rclcpp ros2_unitree_legged_msgs)

# For the nav2 to go1 cmd node.
add_executable(nav2_to_go1_cmd src/nav2_to_go1_cmd.cpp)
ament_target_dependencies(nav2_to_go1_cmd
    rclcpp
    ros2_unitree_legged_msgs
    geometry_msgs
    nav_msgs
)

# Add the new climbing_mode_test.cpp file
add_executable(climbing_mode_test_node src/climbing_mode_test.cpp)
target_link_libraries(climbing_mode_test_node ${EXTRA_LIBS})
ament_target_dependencies(climbing_mode_test_node rclcpp ros2_unitree_legged_msgs)

install(TARGETS
    ros2_udp
    ros2_walk_example
    ros2_position_example
    test_node # Add test_node
    nav2_to_go1_cmd # For the nav2 to go1 cmd node.
    climbing_mode_test_node # Add climbing_mode_test_node
    DESTINATION lib/${PROJECT_NAME}
)
install(FILES
	${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib/libunitree_legged_sdk_${ARCH}.so
     DESTINATION lib}
     )

ament_package()
