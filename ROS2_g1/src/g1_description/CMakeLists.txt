cmake_minimum_required(VERSION 3.8)
project(g1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Instalar arquivos URDF, meshes, config, etc.
install(
  DIRECTORY meshes urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Instalar arquivos URDF na raiz do pacote (compatibilidade)
install(
  FILES g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()