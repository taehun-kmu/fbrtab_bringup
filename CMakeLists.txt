cmake_minimum_required(VERSION 3.8)
project(fbrtab_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# install resource files
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line out if you want to include the copyright check
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line out if you want to include cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
