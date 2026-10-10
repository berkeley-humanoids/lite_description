cmake_minimum_required(VERSION 3.16)
project(lite_description)

find_package(ament_cmake REQUIRED)

# Each variant installs to share/lite_description/robots/<variant>/, so package:// URLs
# and $(find ...) do not depend on the source layout. The cad/ inputs stay out.
install(
  DIRECTORY lite_description/robots
  DESTINATION share/${PROJECT_NAME}
  PATTERN cad EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Per-package LICENSE rather than per-file headers.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
