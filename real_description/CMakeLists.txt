cmake_minimum_required(VERSION 3.8)
project(real_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()