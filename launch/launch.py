from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    # コマンドライン引数の宣言
    output_dir_arg = DeclareLaunchArgument('output_dir', default_value='/home/hurolab/Programs/colcon_ws/data/datafiles', description='CSV出力ディレクトリ')
    steps_arg = DeclareLaunchArgument('steps', default_value='10000', description='総シミュレーションステップ数')
    k2_arg = DeclareLaunchArgument('k_2', default_value='0.5', description='制御パラメータ k2')
    k3_arg = DeclareLaunchArgument('k_3', default_value='-0.3', description='制御パラメータ k3')

    # メイン計算ノードの設定
    main_calculator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='main_calculator_node',
        name='main_calculator_node',
        parameters=[{
            'dt': 0.01,
            'k_2': LaunchConfiguration('k_2'),
            'k_3': LaunchConfiguration('k_3'),
            'k_v': 1.0,
            'k_w': 1.0,
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
        k2_arg,
        k3_arg,
        main_calculator_node,
        csv_logger_node,
        moza_joystick_bridge_node,
        moza_ffb_node
    ])