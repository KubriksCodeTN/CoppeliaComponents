cmake_minimum_required(VERSION 3.8)
project(coppelia_components)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -ltbb) # can't link tbb properly
endif()

#option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  angles
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  coppelia_components
  SHARED
  src/topic_system.cpp
)
#target_compile_options(coppelia_components PUBLIC -static -ltbb)
target_compile_features(coppelia_components PUBLIC cxx_std_20)
target_include_directories(coppelia_components PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/coppelia_components>
)

ament_target_dependencies(
  coppelia_components
  ${HW_IF_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "COPPELIA_COMPONENTS_BUILDING_DLL")

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/coppelia_components
)

install(TARGETS coppelia_components
  EXPORT export_coppelia_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # maybe not needed?
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # GTests
  ament_add_gtest(topic_system_test test/topic_system_test.cpp)
  target_compile_features(topic_system_test PUBLIC cxx_std_20)
  # target_link_libraries(topic_system_test ${PROJECT_NAME}) # this is bad cause of tbb
  ament_target_dependencies(topic_system_test ${HW_IF_INCLUDE_DEPENDS} ros2_control_test_assets)
endif()

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface coppelia_components.xml)
## EXPORTS
ament_export_targets(export_coppelia_components HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()