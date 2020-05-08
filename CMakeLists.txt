cmake_minimum_required(VERSION 3.5)
project(kumo)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SimpleWeb COMPONENTS openssl REQUIRED)

add_library(${PROJECT_NAME}
  "src/${PROJECT_NAME}/http_handler.cpp"
  "src/${PROJECT_NAME}/page_handler.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC "include")

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  SimpleWeb
)

add_executable(daemon "src/daemon.cpp")
target_link_libraries(daemon ${PROJECT_NAME})

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME} DESTINATION "lib")

install(TARGETS daemon DESTINATION "lib/${PROJECT_NAME}")

install(DIRECTORY "share/web" DESTINATION "share/${PROJECT_NAME}")

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  rclcpp
)
ament_package()
