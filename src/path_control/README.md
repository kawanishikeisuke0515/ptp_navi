# path_control

UWB由来の車体中心Poseから直線経路を追従し、`/rov_cmd_vel`に速度指令を出すROS 2パッケージ。

## 設定・起動

`config/path_control.yaml` の `goal_x`・`goal_y` を実際の終点座標[m]で設定する。
両方の指定が必須。未指定・不正値では起動を拒否する。
入力座標系は `world_frame_id`（既定：world）と一致させる。

```bash
cd /path/to/ptp_navi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to path_control
source install/setup.bash
ros2 launch path_control path_control.launch.py
```

別のYAMLを指定する場合：

```bash
ros2 launch path_control path_control.launch.py config_file:=/absolute/path/to/path_control.yaml
```

このlaunchで次をまとめて起動する。

- ZED公式wrapper（zed_wrapper_data_hub経由）
- ZED方位配信
- UWB距離取得・位置推定
- 車体中心位置・姿勢推定
- Twist→RPM変換・Roboteqドライバ
- Jetson電力配信
- Path Control

ZED SDK・公式wrapper、UWBとRoboteqのポート、方位校正、アンカー座標は事前に準備する。
同じノードを別ターミナルから重複起動しない。
ZED起動に10秒以上かかった場合も、入力が正常になればERRORからINITに戻って自動走行する。

```bash
# モーターを起動せずセンサー・速度指令を確認
ros2 launch path_control path_control.launch.py start_locomotion:=false
# Jetson電力監視を省略
ros2 launch path_control path_control.launch.py start_power:=false
# 既存のセンサーを使用し、Path Control単体で起動
ros2 launch path_control path_control.launch.py start_sensors:=false start_locomotion:=false start_power:=false
```

車輪設定は `locomotion_config:=/absolute/path/to/config.yaml`、
中心姿勢補正設定は `robot_pose_config:=/absolute/path/to/config.yaml` で指定可能。

新規有効Poseが10個揃うと自動走行する。deadmanや別の開始操作は不要。
COMPLETE後は停止を維持し、別の目標で走行する場合は設定変更後にノードを再起動する。

## トピック

| 方向 | 名前 | 型 | 内容 |
| --- | --- | --- | --- |
| 入力 | `/uwb/robot_pose` | geometry_msgs/PoseStamped | 車体中心位置・姿勢 |
| 出力 | `/rov_cmd_vel` | geometry_msgs/Twist | 20 Hz、停止・異常遷移時は追加の即時ゼロ指令 |
| 出力 | `/path_control/status` | std_msgs/String | 20 HzのJSON（state, reason, error, init_samples, start, goal） |

入力は有限値、単位Quaternion（ノルム誤差0.01以内）、指定frame_id、単調増加かつ未来でないstampを要求する。
受信停止とstampの古さをともに監視する。統合Poseのため位置・姿勢timeoutの小さい方を適用する。
パラメーターはすべて起動時固定。状態通知のerror=falseとINITへの遷移が異常解除を表す。

## 検証と仕様

```bash
source install/setup.bash
ROS_LOG_DIR=/tmp/path_control_test_logs ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 -m pytest -q src/path_control/test
```

制御・状態遷移の単体テストと、実ROSトピックによる駆動指令・異常停止の統合テストを含む。
実機での座標整合・追従性は未検証。UWBの移動平均遅延はそのまま残る。

- [全体仕様（Markdown）](docs/path_control_parameters_ja.md)
- [全体仕様（HTML）](docs/path_control_parameters_ja.html)
