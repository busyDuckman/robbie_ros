cmake_minimum_required(VERSION 3.16)
project(robbie_description NONE)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The neck's MuJoCo model is generated from config/neck.yaml.
set(NECK_XML ${CMAKE_CURRENT_BINARY_DIR}/neck.xml)
add_custom_command(
  OUTPUT ${NECK_XML}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_neck_mjcf.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/neck.yaml ${NECK_XML}
  DEPENDS scripts/gen_neck_mjcf.py config/neck.yaml
  COMMENT "Generating the neck MJCF")
add_custom_target(neck_mjcf ALL DEPENDS ${NECK_XML})

install(DIRECTORY config urdf mujoco DESTINATION share/${PROJECT_NAME})
install(FILES ${NECK_XML} DESTINATION share/${PROJECT_NAME}/mujoco)

ament_package()
