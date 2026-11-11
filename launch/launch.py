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
    k3_arg = DeclareLaunchArgument('k_3', default_value='1.0', description='制御パラメータ k3')
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='1.0',
        description='経路追従時のROS前進速度',
    )
    path_tracking_arg = DeclareLaunchArgument(
        'use_path_tracking',
        default_value='false',
        description='経路誤差から計算した操舵量をTwistへ使うか',
    )
    use_moza_arg = DeclareLaunchArgument(
        'use_moza',
        default_value='true',
        description='MOZAのJoystick入力ノードを起動するか',
    )
    use_ffb_arg = DeclareLaunchArgument(
        'use_ffb',
        default_value='false',
        description='MOZAのForce Feedbackノードを起動するか',
    )
    override_constant_forward_speed_arg = DeclareLaunchArgument(
        'override_constant_forward_speed',
        default_value='false',
        description='MOZAの前後入力を無視して固定速度で走行するか',
    )
    max_deviation_b_arg = DeclareLaunchArgument(
        'max_deviation_B',
        default_value='512.0',
        description='ジョイスティック左右軸の中心から端までの値',
    )
    max_roll_rad_arg = DeclareLaunchArgument(
        'max_roll_rad',
        default_value='1.0',
        description='ジョイスティック左右端での最大旋回入力 [rad/s相当]',
    )
    invert_steering_arg = DeclareLaunchArgument(
        'invert_steering',
        default_value='true',
        description='ジョイスティック左右軸と旋回方向を反転するか',
    )
    rls_csv_path_arg = DeclareLaunchArgument(
        'rls_csv_path', default_value='', description='RLSオフライン推定に使用するCSV',
    )
    rls_process_csv_arg = DeclareLaunchArgument(
        'rls_process_csv', default_value='false', description='起動時にCSVでRLSを実行するか',
    )
    rlshsc_alpha_arg = DeclareLaunchArgument(
        'rlshsc_alpha', default_value='0.5',
        description='IOC出力とRLS出力の混合係数 alpha',
    )

    # メイン計算ノードの設定
    ioc_calculator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='ioc_calculator_node',
        name='ioc_calculator_node',
        parameters=[{
            'dt': 0.01,
            'k_2': LaunchConfiguration('k_2'),
            'k_3': LaunchConfiguration('k_3'),
            'k_v': 1.0,
            'k_w': 1.0,
            'forward_speed': LaunchConfiguration('forward_speed'),
            'use_path_tracking': LaunchConfiguration('use_path_tracking'),
            'override_constant_forward_speed': LaunchConfiguration(
                'override_constant_forward_speed'
            ),
            'max_deviation_B': LaunchConfiguration('max_deviation_B'),
            'max_roll_rad': LaunchConfiguration('max_roll_rad'),
            'invert_steering': LaunchConfiguration('invert_steering'),
            'publish_control_output': False,
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
            'device': '/dev/input/event5',
            'axis_center': 32767.5,
            'axis_scale': 32767.5,
            'deadzone': 0.02,
            'max_angle_rad': 1.5707963267948966,
            'publish_rate_hz': 100.0,
            'override_constant_forward_speed': LaunchConfiguration(
                'override_constant_forward_speed'
            ),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_moza'))
    )

    moza_ffb_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='moza_ffb_node',
        name='moza_ffb_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ffb'))
    )

    rls_estimator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='rls_estimator_node',
        name='rls_estimator_node',
        parameters=[{
            'feature_indices': [4, 5],
            'phi_ref_index': 11,
            'phi_src_index': 10,
            'csv_path': LaunchConfiguration('rls_csv_path'),
            'process_csv_on_start': LaunchConfiguration('rls_process_csv'),
        }],
        output='screen',
    )

    rlshsc_integrator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='rlshsc_integrator_node',
        name='rlshsc_integrator_node',
        parameters=[{
            'alpha': LaunchConfiguration('rlshsc_alpha'),
            'k_w': 1.0,
            'phi_ioc_index': 10,
            'feature_start_index': 4,
            'feature_count': 2,
        }],
        output='screen',
    )

    return LaunchDescription([
        output_dir_arg,
        steps_arg,
        k2_arg,
        k3_arg,
        forward_speed_arg,
        path_tracking_arg,
        use_moza_arg,
        use_ffb_arg,
        override_constant_forward_speed_arg,
        max_deviation_b_arg,
        max_roll_rad_arg,
        invert_steering_arg,
        rls_csv_path_arg,
        rls_process_csv_arg,
        rlshsc_alpha_arg,
        ioc_calculator_node,
        csv_logger_node,
        moza_joystick_bridge_node,
        moza_ffb_node,
        rls_estimator_node,
        rlshsc_integrator_node
    ])