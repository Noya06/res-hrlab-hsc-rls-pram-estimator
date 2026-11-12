# res-hr-simulate-hsc-ros2-unity

ROS 2 JazzyとUnityをROS-TCP Endpointで接続し、HSC制御を実行するパッケージです。

## 起動方法

### 1. ビルド

```bash
cd /home/hurolab/Programs/hsc_rls_pram_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select res-hr-simulate-hsc-ros2-unity --symlink-install
source install/setup.bash
```

### 2. ROS-TCP Endpointの起動

ターミナル1で実行します。

```bash
cd /home/hurolab/Programs/hsc_rls_pram_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0
```

### 3. ROS制御ノードの起動

ターミナル2で実行します。

```bash
cd /home/hurolab/Programs/hsc_rls_pram_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py \
  use_path_tracking:=false \
  use_moza:=true \
  use_ffb:=false \
  override_constant_forward_speed:=false \
  forward_speed:=1.0
```

その後、Unityを起動してROS-TCP Endpointへ接続します。

### 4. MOZA入力を使用する場合

MOZAのジョイスティック入力を使用し、フォースフィードバックをOFFにする場合:

```bash
cd /home/hurolab/Programs/hsc_rls_pram_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py \
  use_path_tracking:=false \
  use_moza:=true \
  use_ffb:=false \
  k_2:=2.0 \
  k_3:=5.0 \
  override_constant_forward_speed:=false
```

フォースフィードバックを使用する場合は、`use_ffb:=true`に変更します。

### 5. トピック確認

ターミナル3で実行します。

```bash
cd /home/hurolab/Programs/hsc_rls_pram_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /target/robot_errors
```

```bash
ros2 topic echo /real_robot_hum/twist
```

```bash
ros2 topic echo /joystick/joystick_state
```

```bash
ros2 topic echo /joystick/calc_results
```

## 起動パラメータ

| パラメータ | 内容 | 既定値 |
|---|---|---:|
| `k_2` | `e2`に対する制御ゲイン | `2.0` |
| `k_3` | `e3`に対する制御ゲイン | `5.0` |
| `forward_speed` | 経路追従時の前進速度 | `1.0` |
| `use_path_tracking` | 経路追従の有効/無効 | `true` |
| `use_moza` | MOZA入力の有効/無効 | `false` |
| `use_ffb` | MOZAフォースフィードバックの有効/無効 | `false` |
| `override_constant_forward_speed` | MOZAの前後入力を無視して固定速度で走行するか | `true` |
| `max_deviation_B` | ジョイスティック左右軸の中心から端までの値 | `512.0` |
| `max_roll_rad` | ジョイスティック左右端での最大旋回入力 | `1.0` |
| `ioc_damping` | IOCの`e_3`付近の特異点を正則化するダンピング項 | `0.001` |
| `invert_steering` | ジョイスティック左右軸と旋回方向を反転するか | `true` |
| `steps` | CSV記録ステップ数 | `10000` |
| `output_dir` | CSV保存先 | `data/datafiles` |

起動引数の一覧:

```bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py --show-args
```

## 実装内容

### ioc_calculator_node

`/target/robot_errors`から`e2`と`e3`を受信し、制御入力を計算します。

```text
phi_com_y = k_2 * e2 + k_3 * e3
```

IOCは制御用のTwistやFFB指令を直接Publishしません。計算結果を
`/joystick/calc_results`へPublishし、RLS推定ノードとRLS-HSC統合ノードを
経由した出力だけが制御に使われます。

IOCの分母には`ioc_damping`を加えて`e_3`が0に近いときの発散を抑え、
`phi_com_y`は`±max_roll_rad`に制限します。RLS補正後にFFBへ送る
`target_roll`も同じ範囲に制限されます。

`use_path_tracking:=false`の場合は、`/joystick/joystick_state`の入力を使用します。

- `potA`: 前進・後退入力
- `potB`: 操舵入力

`override_constant_forward_speed:=true` の場合は `potA` を使用せず、
`forward_speed` を直ちに送信します。MOZAに触れていなくても前後速度は
開始され、`potB` を受信したときだけ操舵入力が更新されます。

前後・左右ともにジョイスティックだけで操縦する場合は、
`use_path_tracking:=false` と `override_constant_forward_speed:=false` を指定します。
このモードでは `potA` が前後速度、`potB` が左右操舵に使われます。
どちらも中心値は `512`、端までの幅は `512` として `-1〜1` に変換されます。

### moza_joystick_bridge_node

Linux input deviceからMOZAの軸入力を読み取り、
`/joystick/joystick_state`へPublishします。

```text
data[0] = potA
data[1] = potB
```

`override_constant_forward_speed:=true` の固定速度モードでは
`potA=1024`を送信し、MOZAの前後軸を使用しません。
操舵軸は引き続き `potB` から読み取ります。

ジョイスティックだけで操作する場合は、必ず `use_path_tracking:=false` を指定してください。
この場合、回転速度は左右軸の `potB` のみに依存し、前後軸の `potA` や経路誤差
(`e2`, `e3`)からは決まりません。`invert_steering:=true` で左右と旋回方向を反転できます。
左右軸の最大旋回入力は `max_roll_rad` で調整できます。
ジョイスティック操舵モードでは前後軸を使用せず、前進速度を固定値 `1.0` とします。

### moza_ffb_node

`/joystick/target_roll`を購読し、MOZAのフォースフィードバックを制御します。

```text
FFB = p_gain * target_roll + d_gain * target_roll_difference
```

`use_ffb:=false`の場合、このノードは起動しません。
FFB出力には一次ローパスを適用し、急な反力変化を抑えています。
既定値は`force_filter_alpha=0.15`、`max_force_level=20000`です。
これらはFFBの反力だけに作用し、Unityへ送るジョイスティック入力や
ロボットの制御速度は変更しません。

### csv_logger_node

`/joystick/calc_results`と`/real_robot_hum/state`を受信し、CSVへ保存します。

### rls_estimator_node

`/joystick/calc_results`を購読し、`phi_ref=phi_hum_y`、`phi_src=phi_com_y`、
特徴量`[e_2,e_3]`でRLSを逐次更新します。推定パラメータは
`/rls/theta`（`Float64MultiArray`）へ出力されます。現在の配列では、特徴量の列番号は
`[4,5]`、参照入力は`11`、HSC出力は`10`です。

CSVをオフラインで処理する場合:

```bash
ros2 run res-hr-simulate-hsc-ros2-unity rls_estimator_node --ros-args \
  -p csv_path:=/path/to/result.csv \
  -p process_csv_on_start:=true \
  -p csv_output_path:=/path/to/rls_theta.csv
```

`RLSEstimator`は初期共分散`initial_covariance`（既定値`1e3`）と
`initial_parameters`を受け取り、各サンプルでゲイン、パラメータ、共分散を更新します。
更新後の共分散は数値誤差による非対称化を防ぐため対称化されます。

### rlshsc_integrator_node

IOC計算結果とRLS推定値を加算し、FFBへ送る操舵指令を計算します。

```text
target_roll = phi_IOC + (xi^T * theta_hat)
```

`/joystick/calc_results`の`[e_2,e_3]`と`/rls/theta`を使用し、
加算後の値を`/joystick/target_roll`と`/rlshsc/phi_com_y`へ出力します。
一方、`/joystick/joystick_state`から計算した`phi_hum`を
RLS-HSC統合ノードが`/real_robot_hum/twist.angular.z`へ出力します。
`/real_robot_hum/twist`の発行元はRLS-HSC統合ノードだけです。

```bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py
```

記録列:

```text
time,e_2,e_3,v_r,v_xi,phi_com_y,phi_hum_y,y_difference,position_x,position_z,yaw
```

`/real_robot_hum/state`の位置・姿勢データ:

```text
data[0] = Unity position.x
data[1] = Unity position.z
data[2] = ROS yaw
```

既定の保存先:

```text
/home/hurolab/Programs/hsc_rls_pram_ws/data/datafiles
```

### OLSパラメータ推定

`scripts/ols_parameter_identification.py`は、CSVの`e2`、`e3`、
`phi_com_y`から次式の`k_2`,`k_3`を推定します。

```text
phi_com_y = k_2 * e2 + k_3 * e3
```

入力CSVは次のファイル名形式です。

```text
ref_K2_K3.csv
```

例:

```text
ref_2.0_5.0.csv
```

入力CSVを省略して実行した場合は、既定で
`data/ref_2.0_1.0.csv`を読み込みます。別のデータを使う場合は、
`--input-csv`で指定してください。

```bash
python3 scripts/ols_parameter_identification.py \
  --input-csv data/ref_10.0_5.0.csv
```

ファイル名から正解パラメータを取得し、推定値・誤差・RMSE・
各サンプルの予測値を、実行時刻付きのCSVへ保存します。
ファイル名が`ref_K2_K3.csv`形式でない場合も、正解パラメータの比較だけを
スキップして、推定値とRMSEを計算します。
