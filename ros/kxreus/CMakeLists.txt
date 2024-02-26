cmake_minimum_required(VERSION 2.8.3)
project(kxreus)

find_package(catkin REQUIRED
  catkin_virtualenv
  kxr_models
)

catkin_package(
  DEPENDS
  LIBRARIES ${PROJECT_NAME}
)

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

file(GLOB PYTHON_SCRIPT_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
