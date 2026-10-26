cmake_minimum_required(VERSION 3.16)
project(robbie_description NONE)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The neck's MuJoCo model is generated from config/neck.yaml, once per pole (CW or CCW twist,
# seen from above), each with its own scene. The sim picks one at launch.
set(MJCF_FILES)
foreach(POLE CW CCW)
  string(TOLOWER ${POLE} pole)
  set(NECK neck_${pole}.xml)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NECK}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_neck_mjcf.py
            ${CMAKE_CURRENT_SOURCE_DIR}/config/neck.yaml ${POLE} ${CMAKE_CURRENT_BINARY_DIR}/${NECK}
    DEPENDS scripts/gen_neck_mjcf.py config/neck.yaml
    COMMENT "Generating the neck MJCF (${POLE})")
  configure_file(mujoco/scene.xml.in ${CMAKE_CURRENT_BINARY_DIR}/scene_${pole}.xml @ONLY)
  list(APPEND MJCF_FILES ${CMAKE_CURRENT_BINARY_DIR}/${NECK} ${CMAKE_CURRENT_BINARY_DIR}/scene_${pole}.xml)
endforeach()
add_custom_target(neck_mjcf ALL DEPENDS ${MJCF_FILES})

install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})
install(FILES ${MJCF_FILES} DESTINATION share/${PROJECT_NAME}/mujoco)

ament_package()
