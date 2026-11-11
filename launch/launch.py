from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 実験結果とシミュレーション条件
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/hurolab/Programs/hsc_rls_pram_ws/data/datafiles',
        description='CSV出力ディレクトリ',
    )
    steps_arg = DeclareLaunchArgument(
        'steps', default_value='10000', description='総シミュレーションステップ数'
    )
    k2_arg = DeclareLaunchArgument('k_2', default_value='20.0', description='IOC制御ゲイン k2')
    k3_arg = DeclareLaunchArgument('k_3', default_value='0.5', description='IOC制御ゲイン k3')
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='1.0',
        description='経路追従時のROS前進速度',
    )
    path_tracking_arg = DeclareLaunchArgument(
        'use_path_tracking',
        default_value='true',
        description='経路誤差から計算した操舵量をTwistへ使うか',
    )
    use_moza_arg = DeclareLaunchArgument(
        'use_moza',
        default_value='true',
        description='MOZAのJoystick入力ノードを起動するか',
    )
    use_ffb_arg = DeclareLaunchArgument(
        'use_ffb',
        default_value='true',
        description='MOZAのForce Feedbackノードを起動するか',
    )
    override_constant_forward_speed_arg = DeclareLaunchArgument(
        'override_constant_forward_speed',
        default_value='true',
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

    # IOC計算ノード:
    # 誤差からIOC出力を計算し、/joystick/calc_resultsへ中間データを出力する。
    # 制御出力はRLS-HSC統合ノードへ一本化するため、ここでは無効にする。
    ioc_calculator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='ioc_calculator_node',
        name='ioc_calculator_node',
        parameters=[{
            'dt': 0.01,
            # IOCの制御ゲインと速度計算設定
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
            # /real_robot_hum/twistと/joystick/target_rollの二重出力を防止
            'publish_control_output': False,
        }],
        output='screen'
    )

    # CSVログノード:
    # IOC計算結果とUnity状態を収集し、実験結果をCSVへ保存する。
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

    # MOZAジョイスティック入力ノード:
    # Linux input deviceから操舵・前後軸を読み取り、/joystick/joystick_stateへ出力する。
    moza_joystick_bridge_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='moza_joystick_bridge_node',
        name='moza_joystick_bridge_node',
        parameters=[{
            # 使用するデバイスと軸の正規化設定
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

    # MOZA FFBノード:
    # /joystick/target_rollを反力指令へ変換し、ジョイスティックへ送信する。
    # force_filter_alphaとmax_force_levelは反力の急変・過大出力を抑制する。
    moza_ffb_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='moza_ffb_node',
        name='moza_ffb_node',
        parameters=[{
            # FFBの比例・微分ゲインと安全側の出力制限
            'p_gain': 1000.0,
            'd_gain': 0.0,
            'force_filter_alpha': 0.15, # 1.0でフィルタなし
            'max_force_level': 10000, # 最大出力レベル(0-32767)
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ffb'))
    )

    # RLS推定ノード:
    # /joystick/calc_resultsから[e_2, e_3]を特徴量として逐次推定する。
    # 推定値thetaは/rls/thetaへ出力する。
    rls_estimator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='rls_estimator_node',
        name='rls_estimator_node',
        parameters=[{
            # calc_resultsの列: e_2=4, e_3=5, phi_hum=11, phi_ioc=10
            'feature_indices': [4, 5],
            'phi_ref_index': 11,
            'phi_src_index': 10,
            'csv_path': LaunchConfiguration('rls_csv_path'),
            'process_csv_on_start': LaunchConfiguration('rls_process_csv'),
        }],
        output='screen',
    )

    # RLS-HSC統合ノード:
    # phi_target = phi_IOC + xi^T * theta_hatを計算する。
    # target_rollはFFBへ、phi_humはJoystick入力からUnityへ送る。
    rlshsc_integrator_node = Node(
        package='res-hr-simulate-hsc-ros2-unity',
        executable='rlshsc_integrator_node',
        name='rlshsc_integrator_node',
        parameters=[{
            # IOC/RLS入力の列番号と速度出力設定
            'k_w': 1.0,
            'phi_ioc_index': 10,
            'feature_start_index': 4,
            'feature_count': 2,
            'joystick_center': 512.0,
            'joystick_max_deviation': LaunchConfiguration('max_deviation_B'),
            'max_roll_rad': LaunchConfiguration('max_roll_rad'),
            'invert_steering': LaunchConfiguration('invert_steering'),
        }],
        output='screen',
    )

    # 引数を先に登録し、その後に各ノードを起動する。
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
        ioc_calculator_node,
        csv_logger_node,
        moza_joystick_bridge_node,
        moza_ffb_node,
        rls_estimator_node,
        rlshsc_integrator_node
    ])