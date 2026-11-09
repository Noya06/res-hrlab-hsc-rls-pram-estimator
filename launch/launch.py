import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # コマンドライン引数の宣言
    output_dir_arg = DeclareLaunchArgument('output_dir', default_value='/home/hurolab/Programs/colcon_ws/data/datafiles', description='CSV出力ディレクトリ')
    steps_arg = DeclareLaunchArgument('steps', default_value='10000', description='総シミュレーションステップ数')

    # メイン計算ノードの設定
    main_calculator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='main_calculator_node',
        name='main_calculator_node',
        parameters=[{
            'x0': 0.0,
            'y0': -1.0,
            'theta0_deg': 0.0,
            'dt': 0.01,
            'alpha': 0.5,
            'k_2': 60.0,
            'k_3': 10.0,
            'c_0': 0.0,
            'k_v': 1.0,
            'k_w': 1.0
        }],
        output='screen'
    )

    # CSV記録ノードの設定
    csv_logger_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='csv_logger_node',
        name='csv_logger_node',
        parameters=[{
            'steps': LaunchConfiguration('steps'),
            'output_dir': LaunchConfiguration('output_dir')
        }],
        output='screen'
    )

    optimization_bridge_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='optimization_bridge_node',
        name='optimization_bridge_node',
        output='screen'
    )

    # MOZA AB9 FFB Base からのスティック入力を ROS トピックへ反映するノード
    moza_joystick_bridge_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='moza_joystick_bridge_node',
        name='moza_joystick_bridge_node',
        parameters=[{
            'device': '/dev/input/event4',
            'axis_center': 32767.5,
            'axis_scale': 32767.5,
            'deadzone': 0.02,
            'max_angle_rad': 1.5707963267948966,
            'publish_rate_hz': 100.0
        }],
        output='screen'
    )

    moza_ffb_node = Node(
    package='res-hr-simulate-hsc-ros2-unity',
    executable='moza_ffb_node',
    name='moza_ffb_node',
    output='screen'
    )

    return LaunchDescription([
        output_dir_arg,
        steps_arg,
        main_calculator_node,
        csv_logger_node,
        optimization_bridge_node,
        moza_joystick_bridge_node,
        moza_ffb_node
    ])