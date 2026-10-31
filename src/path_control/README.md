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

## 実験ログ

Launchは既定で `path_control_logger` も起動する。保存先は
`~/path_control_logs/<UTC日時>_<一意なID>/`。`start_logger:=false` で無効化できる。
OptiTrack配信ノードはこのLaunchでは起動しない。配信済みのPoseを購読する。

```bash
# 別のロガー設定を指定
ros2 launch path_control path_control.launch.py logger_config:=/absolute/path/to/logger.yaml
# ロガー単体で起動
ros2 run path_control path_control_logger --ros-args \
  --params-file /absolute/path/to/logger.yaml
```

設定の雛形は `config/path_control_logger.yaml`。OptiTrackのRigidBody名や、
制御側のトピック名を変更した場合はロガー側も合わせる。

| ファイル | 内容 |
| --- | --- |
| `status.csv` | 制御状態・理由・始点A・目標B・受信したJSON全文 |
| `events.csv` | Aの確定・破棄イベントと走行区間番号 |
| `cmd_vel.csv` | ローバー指令 linear.x/y [m/s]、angular.z [rad/s] |
| `optitrack.csv` | 元の位置xyz [m]、Quaternion、座標系 |
| `uwb_position.csv` | 三辺測位位置xy [m]、valid、デバイス時刻 |
| `robot_pose.csv` | 制御入力となる補正後の車体中心位置・姿勢 |
| `timeline.csv` | 既定20 Hzで各ストリームの最新値・受信有無・鮮度を並べた表 |
| `metadata.yaml` | ロガー設定、単位、各ノードから取得した実パラメータ |

各受信CSVは受信ごとに1行保存する。共通列はROS受信時刻・単調増加時計の受信時刻
（いずれも整数ns）、元のheader時刻・frame_id、値の有効性、走行区間番号。
headerのない指令・statusの元時刻は空欄。未受信の値も空欄とし、ゼロで埋めない。
`timeline.csv` は補間・厳密な同期を行わず、最新受信値を保持する。
`age_sec` は単調増加時計による受信後経過時間で、既定0.6秒を超えると `stale=True`。
元の計測時刻の遅れは `source_stamp_ns` とROS受信時刻から別途確認する。

Aは `/path_control/status` の `start` を記録し、ロガーでは再推定しない。
Aが未確定から確定、または別座標へ変わると `segment_id` が増える。
ERROR復帰時のA破棄→再確定は、同じ座標でも別区間になる。
区間番号はロガーが観測した順序による。途中起動した場合の開始前データは復元できない。
各最新値にも受信時の区間番号が残るため、区間をまたいで保持された値を識別できる。

実パラメータは制御・UWB測位・車体Pose・ZED方位・RPM変換の各ノードから、
サービス接続後に非同期で1回取得する。アンカー位置・フィルタ・車体補正・目標Bなどを
含み、取得時刻も保存する。途中の設定変更履歴は対象外。
正常終了時は取得できなかったノードを `missing_parameter_snapshots` に記録する。
センサーが未接続でも他ストリームの記録は継続する。

OptiTrackはbest effort、他はreliableで購読する。ログは毎秒flushし、正常終了時にcloseする。
強制終了・電源断では最後のバッファが失われる可能性がある。
座標変換・OptiTrack剛体と車体中心の位置合わせは行わないため、
座標系と取り付け位置を揃えてから誤差を評価する。
