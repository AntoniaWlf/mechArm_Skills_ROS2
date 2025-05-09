cmake_minimum_required(VERSION 3.8)
project(mecharm_skills)

# 1) Finde ament- und ROS-2-Pakete
find_package(ament_cmake REQUIRED)
find_package(rclpy          REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(std_srvs       REQUIRED)
find_package(launch         REQUIRED)
find_package(launch_ros     REQUIRED)

# 2) Installiere dein Python-Package (wie in setup.py definiert)
ament_python_install_package(${PROJECT_NAME})

# 3) Kopiere alle Launch-Dateien ins Install-Share-Verzeichnis
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# 4) Exportiere als ament-Package
ament_package()