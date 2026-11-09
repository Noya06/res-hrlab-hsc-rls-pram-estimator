# res-he-simulate-hsc-ros2-unity
HSC勉強のためのシミュレーション．ROS2とUnityを使用．M1．

## 起動コマンド
以下は、ROS 2 + Unity を組み合わせた通常起動と最適化評価の流れを、実際にそのまま実行できる形で整理した例です。

### 1. ワークスペースに入る
```bash
cd /home/hurolab/Programs/colcon_ws
source /opt/ros/jazzy/setup.bash
```

### 2. 依存パッケージを取得する
必要に応じて、ROS側の依存を src 配下に入れます。

```bash
cd /home/hurolab/Programs/colcon_ws/src
git clone https://github.com/roverrobotics-forks/serial-ros2.git
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

### 3. パッケージをビルドする
```bash
cd /home/hurolab/Programs/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select res-hr-simulate-hsc-ros2-unity --symlink-install
source install/setup.bash
```

### 4. ROS 側のノードを起動する
ターミナル1で実行:
```bash
cd /home/hurolab/Programs/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py
```

この launch では、既存の制御ノードと必要に応じて最適化用の bridge ノードが起動します。

### 5. ROS-TCP-Endpoint を起動する
ターミナル2で実行:
```bash
cd /home/hurolab/Programs/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

これで ROS-TCP Endpoint が待ち受けます。Unity 側はこの endpoint に合わせて接続先 IP / port を設定します。既定の port は 10000 です。

### 6. Unity を起動して接続する
- Unity プロジェクトを起動する
- ROS TCP Endpoint の設定に従って接続先 IP / port を合わせる
- Unity 側が購読する topic 名と、ROS 側が publish する topic 名を一致させる
- 通常モードで動作させると、既存のロボット制御・推定・表示が動く

#### 接続できない時の確認ポイント
- ros_tcp_endpoint が 10000 番で起動しているか
- Unity 側が rosbridge(9090) ではなく ROS-TCP Endpoint に接続しているか
- `main_calculator_node` が publish する topic と Unity 側の購読 topic が一致しているか
- 最初の Twist を受け取るまで移動処理が待機しているので、受信フラグが立つか

このパッケージ側では、受信待ちの間は待機ログを出します。もし動かない場合は、トピック不一致か接続先不一致を最優先で確認してください。

### 7. 手動モードでの動作確認
```bash
ros2 topic list | grep -E "joystick|real_robot|optimization"
ros2 topic echo /joystick/calc_results
```
必要に応じて Joystick / Arduino 側の入力が ROS に届いていることを確認する。

### HSC制御と実測データの記録

`main_calculator_node`はUnityの`/target/robot_errors`から`e2,e3`を受信し、
次の機械制御量を計算して`/joystick/target_roll`へ送信します。

```text
U_com = phi_com_y = k2 * e2 + k3 * e3
```

Unityの`DummyJoystickPublisher`を有効にすると、この値を人間が操作していない
Joystick入力として`/joystick/joystick_state`へ返します。MOZAを使う場合は従来どおり
`moza_joystick_bridge_node`を使用できます。ROS2はJoystick値から`phi_hum_y`を計算し、
`/real_robot_hum/twist`としてUnityへ送信します。

経路追従を行う場合は、起動時の`use_path_tracking`（既定値`true`）により、
`k_2 * e2 + k_3 * e3`を`Twist.angular.z`へ直接使用します。
ジョイスティック操舵へ戻す場合は、`use_path_tracking:=false`を指定します。
経路追従時の前進速度は`forward_speed`（既定値`1.0`）です。
Unity側の座標変換で前後が逆になる場合は、`linear_x_sign:=-1.0`で
`Twist.linear.x`の符号を反転できます。

```bash
export PATH="/home/hurolab/MATLAB/bin:$PATH"
cd /home/hurolab/Programs/colcon_ws
source /opt/ros/jazzy/setup.bash
matlab -batch "cd('/home/hurolab/Programs/colcon_ws/src/res-hr-simulate-hsc-ros2-unity/scripts'); bayesopt_driver"
```

MATLAB のコマンドウィンドウから直接実行する場合:
```matlab
cd('/home/hurolab/Programs/colcon_ws/src/res-hr-simulate-hsc-ros2-unity/scripts');
bayesopt_driver
```

または:
```matlab
addpath('/home/hurolab/Programs/colcon_ws/src/res-hr-simulate-hsc-ros2-unity/scripts');
run('bayesopt_driver.m');
```

> 最終設計では、Python の簡易評価スクリプトは使わず、Unity の評価モードが実評価の本体である。MATLAB は外側の探索ループとして rho を ROS 経由で送り、結果を受け取る。

### 9. rho を ROS へ送る
MATLAB 側の driver は、候補 rho を以下の ROS topic に送る想定です。

```bash
ros2 topic pub /optimization/rho_cmd std_msgs/msg/Float64MultiArray "{data: [20.0, 5.0, 0.01]}"
```

この topic で受け取った値は、最適化 bridge 側で既存のパラメータへ反映される設計です。

### 10. Unity 評価モードで固定シナリオを回す
Unity 側では、通常モードと評価モードを切り替える。
- 通常モード: 実時間操縦・可視化
- 評価モード: 固定シナリオを短時間で再生し、y_com を出力

評価用の CSV 例:
```bash
cd /home/hurolab/Programs/colcon_ws
python3 - <<'PY'
import csv
with open('tmp_human_demo.csv','w',newline='') as f:
    w=csv.writer(f)
    w.writerow(['time','r1','r2','y1','y2'])
    for i in range(10):
        t=i*0.01
        w.writerow([t, 0.0, 0.0, 0.0, 0.0])
PY
```

### 11. 最適化結果を確認する
MATLAB 側で J を計算し、次の候補を生成して繰り返す。最終的に、最良の rho を採用する。

```bash
ros2 topic echo /optimization/rho_cmd
```

### 11.1 1試行ごとのリセット
ここでいう 1 試行は 10000 行の記録を指す。

各試行の終了後は、MATLAB 側から以下のトピックを 1 回 publish して初期状態へ戻す。

```bash
ros2 topic pub --once /optimization/reset_episode std_msgs/msg/Bool "{data: true}"
```

この reset は ROS 側の低層の初期化であり、手動/自動のようなモード分岐は導入しない。

### 12. 一連の流れまとめ
```bash
# 1) ROS 環境の読み込み
cd /home/hurolab/Programs/colcon_ws
source /opt/ros/jazzy/setup.bash

# 2) ビルド
colcon build --packages-select res-hr-simulate-hsc-ros2-unity --symlink-install
source install/setup.bash

# 3) ROS ノード起動
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py

# 4) Unity 連携側起動
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# 5) MATLAB で bayesopt 実行
matlab -batch "cd('/home/hurolab/Programs/colcon_ws/src/res-hr-simulate-hsc-ros2-unity/scripts'); bayesopt_driver"
```

この手順により、通常の ROS2 + Unity の運用と、bayesopt による外部最適化を同じまとまりの中で切り替えて実行できる。
## Joystick(Arduino)との通信に関して
serial-ros2パッケージを用いる
https://github.com/roverrobotics-forks/serial-ros2
をsrcにクローンすること

## unityとの接続に関して
https://github.com/Unity-Technologies/ROS-TCP-Endpoint
をsrcにクローンすること

## unity側の実装について
https://www.sra.co.jp/ros/column/ros2-unity-01.html
を参考に実装
https://qiita.com/noya_06/items/8b2f35ee72f1bd7c7692
上記の問題が発生したため、そこだけ変更

## UnityのC#側で座標変換を実装
位置の変換:
ROSの「X（前）」は Unityの「Z（前）」に対応します。
ROSの「Y（左）」は Unityの「X（左）」に対応するため、Unityの右方向を正とする軸に合わせるには符号を反転して -rosY になります。

回転の変換:
ROSは右上方向（反時計回り）が正ですが、Unityは上から見て時計回りが正になります。また、ROSのZ軸回転はUnityのY軸（上方向）の回転に対応するため、符号を反転（-rosTheta）して度数法に変換します。

// ROS座標系でのシミュレート値 (X: 前方, Y: 左方, Z軸回転: 反時計回りが正)

## 最適化連携（bayesopt + Unity）
現在の構成では、Pythonの高速シミュレータではなく、既存のUnityシミュレーションを本体として用いる。
MATLABのbayesoptで候補パラメータ rho = [K2, K3, alpha] を生成し、ROS経由でUnityへ与える。
Unity側では評価モードに切り替え、固定シナリオを短時間で実行し、y_com を出力する。
MATLAB側では y_hum と y_com の差分二乗和

J(rho) = sum_{k=1}^{N} || y_hum(k) - y_com(k) ||^2

を計算して、bayesoptへ返す。

### 連携の流れ
1. 人間がJoystickで30秒程度操作し、r と y_hum を記録する
2. MATLABでbayesoptを起動し、rho の候補を生成する
3. ROS topic /optimization/rho_cmd に rho を送る
4. Unity側の評価モードで固定シナリオを短時間実行する
5. Unity側が y_com をCSVまたはROS topicへ出力する
6. MATLAB側で J を計算し、次の候補を探索する
7. 30回程度の反復後に最良の rho を最終ゲインとして採用する

### 追加予定のROS topic
- /optimization/rho_cmd
  - 形式: std_msgs/Float64MultiArray
  - 内容: [K2, K3, alpha]

### 重要な設計方針
- 既存の制御ロジックやUnityのシミュレーション本体は変更しない
- 最適化のための処理だけを外側に追加する
- 実際の評価はUnityで行い、Pythonの簡易シミュレータは使わない
- 評価中は描画を落とし、headlessまたはlow-renderで高速実行する

### MATLAB側の役割
- bayesopt による代理モデル生成
- 獲得関数の最適化
- rho の候補生成
- J の返却
- 反復回数の管理

### Unity側の役割
- /optimization/rho_cmd を受信
- 評価モード時に固定シナリオを短時間で再生
- 制御対象と同じロジックで y_com を生成
- y_com をCSV/ROS topicへ出力
- 通常モードでは従来のリアルタイムシミュレーションを維持

### 既知の制約
- 既存の制御コードを大きく書き換えない
- クラス設計を複雑化しない
- 最適化用の追加ロジックは最小差分で実装する
- 収束が失敗した候補 rho は J = Inf として扱う

## 実行手順のイメージ
- ターミナル1: ROS2とUnity連携の通常起動
- ターミナル2: ROS-TCP-Endpoint起動
- MATLAB: bayesoptの最適化スクリプト実行
- Unity: 評価モードでアプリケーションを走行させ、結果を保存

これにより、通常の手動操作と最適化評価を同じシステム上で切り替えながら運用できる。


## 起動コマンド
ターミナル1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py

ターミナル2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0



久々に起動した際には
ip a
にてアドレスの確認、unityへの入力
と
evtest
にてjoystickのusbの情報の確認とmainへの入力を


手動操作時にはjoystickを接続し、csファイルの方で時間スケールを1.0fに
unity上にてevaluationModeのチェックとDummyJoystickのチェックを外す
ターミナル2つで上記2種類のコマンドを実行
main_calcにて    this->declare_parameter<bool>("use_csv_errors", false);に
joystickのevent番号、.cと.pyと.

## OLSパラメータ同定の検証

CSV loggerはROS2の同じ制御サンプルについて、`e2`、`e3`、`phi_com_y`、
`phi_hum_y`を保存します。Pythonは記録済みの値から`y = phi_hum_y - phi_com_y`
を計算し、OLSで`k2`と`k3`を推定します。
検証スクリプトは[`scripts/ols_parameter_identification.py`](scripts/ols_parameter_identification.py)です。

```bash
python3 scripts/ols_parameter_identification.py \
  --input-csv /path/to/ros2_result.csv \
  --output-csv data/ols_identification.csv \
  --plot data/ols_fit.png
```

ROS2を起動してUnityでDummyJoystickを有効にした後、保存されたCSVを処理します。

```bash
ros2 launch res-hr-simulate-hsc-ros2-unity launch.py \
  k_2:=0.5 k_3:=-0.3 \
  output_dir:=/path/to/data
```

入力CSVはUnityから受信したe2/e3をROS2が処理したログであり、
`e_2,e_3,phi_com_y,phi_hum_y,y_difference`を含みます。
`--input-csv`を省略すると、スクリプト単体の動作確認用にデモデータを生成します。

シミュレーション時にはjoystickは抜いておき、csの方で時間スケールを5.0fに
unity上でチェックを2つ入れ
ターミナルで2つに加え、.mファイルを実行
main_calcにて    this->declare_parameter<bool>("use_csv_errors", true);に
mainとmの両方にcsv名称を書く必要あり、注意