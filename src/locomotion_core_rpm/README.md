# locomotion_core_rpm

既存の locomotion_core と同じ2ノード構成で、TwistをモーターRPMへ変換し、Roboteqへ `!S` を送信するROS 2パッケージ。

## 設定

`config/locomotion_core_rpm.yaml` を編集する。

- `max_motor_rpm`：各モーターの許容RPMを4要素で指定する。配布YAMLでは全輪300 RPMに設定する。YAMLを使用せず起動する場合は明示的な指定が必要で、未指定の0は起動を拒否する。変換ノードとドライバーに同じ値が適用される。
- `gear_ratios`：モーター回転数÷車輪回転数。初期値は全輪1.0。
- 寸法はmで設定する。車輪半径0.0635、長さ0.205・0.170を使用する。
- `motor_directions`：配列順に各輪の方向を+1/-1で指定。初期値は既存と同じ全輪-1。
- 配列順は前側CH1、前側CH2、後側CH1、後側CH2。

RoboteqをClosed Loop Speedに設定し、機種の `!S` 対応と速度フィードバックの軸基準を合わせる。旧ドライバーは終了させ、同一ポートを同時に使用しない。既存の rover_velocity も停止しておく。

## ビルド・起動

ワークスペースで実行する。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select locomotion_core_rpm
source install/setup.bash
ros2 launch locomotion_core_rpm wheel_control.launch.py
```

別の設定ファイルを使う場合：

```bash
ros2 launch locomotion_core_rpm wheel_control.launch.py config:=/absolute/path/to/config.yaml
```

RPM直接入力の試験ではドライバーだけ起動する。

```bash
ros2 launch locomotion_core_rpm rpm_driver.launch.py
```

どちらも起動時に全輪停止し、新しいTwistまたはRPM入力で駆動する。RPMトピックは `/rov/motor_rpm`（Float32MultiArray、4要素）。Twistは `/rov_cmd_vel`（m/s、rad/s）。同じトピックの発行元は一つにする。

wheel_control起動時の停止操作（ゼロ指令を継続送信、Ctrl+Cで終了）：

```bash
ros2 topic pub --rate 20 /rov_cmd_vel geometry_msgs/msg/Twist '{}' 
```

RPM入力が0.5秒途絶えると停止する。新しい有効なRPM入力で再開する。Twist入力が途絶えると変換ノードがゼロRPMを発行する。通信障害はラッチし、原因を解消してドライバーを再起動する。

## 動作例

前進0.1 m/s、横移動・旋回ゼロ、減速比1では、4輪とも約15.038 RPM。初期の方向係数と整数化により、各CHへ `!S <CH> -15_` を送る（RPM上限がこの値以上の場合）。

## 検証

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=".:$PYTHONPATH" python3 -m pytest -q test
```

純粋な換算・停止処理はモックで、ROSノードとpyserialの接続は仮想シリアルポートで検証する。テストは実機ポートを使用しない。

## 現時点の制約

実機の回転方向・速度追従・車輪配置は未検証。送信成功はコントローラの受理や実測RPMへの到達を保証しない。シリアル応答の解析、実測RPMの取得、自動再接続は未実装。プロセス停止・通信断時に備え、コントローラ側のウォッチドッグを設定する。

[仕様書（Markdown）](docs/locomotion_core_rpm_spec_ja.md) / [仕様書（HTML）](docs/locomotion_core_rpm_spec_ja.html)
