cmake_minimum_required(VERSION 3.5)
project(gui)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(topic_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  gui/simple_GUI.py
  gui/GUI.py
  gui/GUI_test_pub.py
  gui/GUI_theme.py
  gui/helper_classes.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()