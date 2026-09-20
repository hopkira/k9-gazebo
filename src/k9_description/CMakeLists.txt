cmake_minimum_required(VERSION 3.8)
project(k9_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the latest model from its existing source location, not the legacy URDF.
install(FILES ../k9_robot/src/description/k9.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/model)

ament_package()
