from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    # コマンドライン引数の宣言
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/hurolab/Programs/hsc_rls_pram_ws/data/datafiles',
        description='CSV出力ディレクトリ',
    )
    steps_arg = DeclareLaunchArgument('steps', default_value='10000', description='総シミュレーションステップ数')
    k2_arg = DeclareLaunchArgument('k_2', default_value='2.0', description='制御パラメータ k2')
    k3_arg = DeclareLaunchArgument('k_3', default_value='5.0', description='制御パラメータ k3')
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='1.0',
        description='経路追従時のROS前進速度',
    )
    linear_x_sign_arg = DeclareLaunchArgument(
        'linear_x_sign',
        default_value='1.0',
        description='Unity座標系に合わせるlinear.xの符号（1または-1）',
    )
    path_tracking_arg = DeclareLaunchArgument(
        'use_path_tracking',
        default_value='true',
        description='経路誤差から計算した操舵量をTwistへ使うか',
    )
    use_moza_arg = DeclareLaunchArgument(
        'use_moza',
        default_value='false',
        description='MOZAのJoystick/FFBノードを起動するか',
    )

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
            'forward_speed': LaunchConfiguration('forward_speed'),
            'linear_x_sign': LaunchConfiguration('linear_x_sign'),
            'use_path_tracking': LaunchConfiguration('use_path_tracking'),
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
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_moza'))
    )

    moza_ffb_node = Node(
    package='res-hr-simulate-hsc-ros2-unity',
    executable='moza_ffb_node',
    name='moza_ffb_node',
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_moza'))
    )

    return LaunchDescription([
        output_dir_arg,
        steps_arg,
        k2_arg,
        k3_arg,
        forward_speed_arg,
        linear_x_sign_arg,
        path_tracking_arg,
        use_moza_arg,
        main_calculator_node,
        csv_logger_node,
        moza_joystick_bridge_node,
        moza_ffb_node
    ])