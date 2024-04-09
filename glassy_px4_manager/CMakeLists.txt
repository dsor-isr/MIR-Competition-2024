cmake_minimum_required(VERSION 3.5)
project(glassy_px4_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glassy_msgs REQUIRED)

# get glassy_messages as well as px4 messages
find_package(px4_msgs REQUIRED)


#GLOBING
file(GLOB_RECURSE SOURCE_FILES src/*.cpp)           #------------ important to work


add_executable(glassy_px4_manager src/main.cpp ${SOURCE_FILES})

target_compile_features(glassy_px4_manager PRIVATE cxx_std_20)

ament_target_dependencies(glassy_px4_manager rclcpp std_msgs px4_msgs glassy_msgs)

install(TARGETS glassy_px4_manager
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
