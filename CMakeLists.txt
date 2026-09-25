cmake_minimum_required(VERSION 3.30)

project(rmcs-navigation-behaviors)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED)
target_compile_features(
    ${PROJECT_NAME}
    PUBLIC
        cxx_std_23
)
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        src/example_behavior.cc
)
target_include_directories(
    ${PROJECT_NAME}
    SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/src/
        ${nav2_behaviors_INCLUDE_DIRS}
        ${nav2_core_INCLUDE_DIRS}
        ${nav2_msgs_INCLUDE_DIRS}
)
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        ${nav2_behaviors_LIBRARIES}
)

pluginlib_export_plugin_description_file(
    nav2_core behavior_plugin.xml
)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/
)

find_package(ament_cmake REQUIRED)
ament_package()
