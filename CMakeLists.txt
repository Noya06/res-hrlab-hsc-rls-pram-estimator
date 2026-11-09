cmake_minimum_required(VERSION 3.8)
project(res-hr-simulate-hsc-ros2-unity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# 1. 依存パッケージの検索
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ------------------------------------------------------------------------------
# 2. 実行可能ファイル（Executable）のビルド設定
# ------------------------------------------------------------------------------

# メイン計算ノード
add_executable(main_calculator_node src/main_calculator_node.cpp)
ament_target_dependencies(main_calculator_node
  rclcpp
  std_msgs
  geometry_msgs
)

# CSVログ記録ノード
add_executable(csv_logger_node src/csv_logger_node.cpp)
ament_target_dependencies(csv_logger_node
  rclcpp
  std_msgs
)

# MOZA ジョイスティック ブリッジノード
add_executable(moza_joystick_bridge_node src/moza_joystick_bridge_node.cpp)
ament_target_dependencies(moza_joystick_bridge_node
  rclcpp
  std_msgs
)

# ------------------------------------------------------------------------------
# 3. インストール設定 (ros2 run / ros2 launch で参照できるように配置)
# ------------------------------------------------------------------------------

# 実行可能バイナリのインストール
install(TARGETS
  main_calculator_node
  csv_logger_node
  moza_joystick_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch ファイルディレクトリのインストール (存在する場合)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# OLS検証スクリプトのインストール (存在する場合)
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
  OPTIONAL
)

# Pythonスクリプトのインストール設定
install(PROGRAMS
  src/moza_ffb_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moza_ffb_node
)

# ------------------------------------------------------------------------------
# 4. ament パッケージマクロの呼出
# ------------------------------------------------------------------------------
ament_package()