# ZED Heading Publisher 設計仕様書

Version: 0.2 draft
Date: 2026-09-01
Status: draft

## 1. 概要

`zed_heading_publisher` は、ZED data hub が publish する magnetometer topic を subscribe し、hard-iron 補正とゼロ点補正を行ったうえで、ロボット yaw 角を ROS 2 topic に publish するノードである。

本ノードは方位角の算出と publish を担当する。速度指令、自己位置推定、センサフュージョン、地図座標系への変換は本設計の対象外とし、後段ノードで実施する。

## 2. 設計方針

- ZED2i camera device access は `zed_wrapper_data_hub` が起動する公式 `zed_wrapper` に集約する。
- 本ノードは magnetometer topic の subscribe、磁場補正、yaw 算出、publish を担当する。
- hard-iron 補正値とゼロ点補正値は ROS parameter として外部から変更できるようにする。
- 制御で使う角度は `[-pi, pi)` の rad として publish する。
- 実験時の確認と rosbag 解析のため、raw 磁場値と補正後磁場値も同じ message に含める。
- 左旋回時に yaw が増加する向きを正方向とする。
- ZED data hub から magnetometer data を受信できない場合、古い値を再 publish しない。

## 3. パッケージ構成

### 3.1 実行パッケージ

```text
zed_heading_publisher
```

想定する主要ファイル:

```text
zed_heading_publisher/
  package.xml
  setup.py
  setup.cfg
  resource/zed_heading_publisher
  zed_heading_publisher/
    __init__.py
    zed_heading_publisher.py
  launch/
    zed_heading_publisher.launch.py
  config/
    zed_heading_publisher.yaml
  docs/
    zed_heading_publisher_spec_ja.md
```

### 3.2 メッセージパッケージ

カスタムメッセージは別パッケージに分離する。

```text
zed_interfaces
```

想定する主要ファイル:

```text
zed_interfaces/
  package.xml
  CMakeLists.txt
  msg/
    ZedHeading.msg
```

## 4. ノード設計

| 項目 | 内容 |
| --- | --- |
| Package | `zed_heading_publisher` |
| Node | `zed_heading_publisher` |
| Executable | `zed_heading_publisher` |
| Main file | `zed_heading_publisher/zed_heading_publisher.py` |

### 4.1 Responsibilities

`zed_heading_publisher` は以下を担当する。

1. ROS parameter の宣言、取得、validation
2. magnetometer topic の subscribe
3. magnetometer raw field の取得
5. X-Z 平面の hard-iron 補正
6. 磁気角 `magnetic_heading_deg` の算出
7. ロボット yaw `robot_yaw_deg`, `robot_yaw_rad` の算出
8. `zed_interfaces/msg/ZedHeading` の生成
9. `/zed/heading` への publish

## 5. Subscribe Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/zed2i/zed_node/imu/mag` | `sensor_msgs/msg/MagneticField` | ZED wrapper が publish する magnetometer data |

topic 名は `mag_topic` parameter で変更可能とする。

本ノードは ZED SDK の `pyzed.sl.Camera` から直接 sensor data を取得しない。

## 6. Publish Topic

| Topic | Type | 用途 |
| --- | --- | --- |
| `/zed/heading` | `zed_interfaces/msg/ZedHeading` | raw 磁場値、補正後磁場値、磁気角、ロボット yaw を publish する |

### 6.1 Message Fields

```text
std_msgs/Header header

float32 raw_x
float32 raw_z

float32 corrected_x
float32 corrected_z

float32 magnetic_heading_deg
float32 robot_yaw_deg
float32 robot_yaw_rad

bool valid
```

| Field | Type | Requirement |
| --- | --- | --- |
| `header` | `std_msgs/Header` | publish 時刻と `frame_id` を格納すること。 |
| `raw_x` | `float32` | ZED2i magnetometer の未補正 X 軸磁場値を格納すること。 |
| `raw_z` | `float32` | ZED2i magnetometer の未補正 Z 軸磁場値を格納すること。 |
| `corrected_x` | `float32` | `raw_x - center_x` を格納すること。 |
| `corrected_z` | `float32` | `raw_z - center_z` を格納すること。 |
| `magnetic_heading_deg` | `float32` | 補正後 X-Z 平面から算出した磁気角[deg]を格納すること。範囲は `[-180, 180)` とする。 |
| `robot_yaw_deg` | `float32` | ゼロ点補正後のロボット yaw[deg]を格納すること。範囲は `[-180, 180)` とする。 |
| `robot_yaw_rad` | `float32` | `robot_yaw_deg` を rad に変換した値を格納すること。範囲は `[-pi, pi)` とする。 |
| `valid` | `bool` | センサ取得と角度算出が成功した場合 `true` とすること。 |

## 7. Parameters

| Parameter | Type | Default | Range / Constraint | 用途 |
| --- | --- | --- | --- | --- |
| `mag_topic` | `string` | `/zed2i/zed_node/imu/mag` | non-empty | subscribe する magnetometer topic |
| `center_x` | `double` | `-0.629279` | finite value | 360 度回転データから求めた磁場中心 X |
| `center_z` | `double` | `-0.874388` | finite value | 360 度回転データから求めた磁場中心 Z |
| `soft_iron_matrix_00` | `double` | `1.0` | finite value | soft-iron 補正行列 row 0 column 0 |
| `soft_iron_matrix_01` | `double` | `0.0` | finite value | soft-iron 補正行列 row 0 column 1 |
| `soft_iron_matrix_10` | `double` | `0.0` | finite value | soft-iron 補正行列 row 1 column 0 |
| `soft_iron_matrix_11` | `double` | `1.0` | finite value | soft-iron 補正行列 row 1 column 1 |
| `zero_heading_deg` | `double` | `39.7` | finite value | ロボット実 yaw = 0 deg のときの磁気角 |
| `publish_rate_hz` | `double` | `20.0` | `> 0.0` | `/zed/heading` の最大 publish rate |
| `frame_id` | `string` | `zed2i_mag` | non-empty | message header の frame id |
| `invert_yaw` | `bool` | `false` | `true` or `false` | yaw 符号反転が必要な場合に使用する |
| `magnetic_field_scale` | `double` | `1000000.0` | finite value | `sensor_msgs/msg/MagneticField` の Tesla 値を microtesla 相当へ変換するscale |
| `diagnostic_log_interval_sec` | `double` | `1.0` | `>= 0.0` | 磁気センサ値、scale後raw値、補正後値、yawを診断logに出す周期。`0.0` で無効 |
| `raw_x_axis` | `string` | `y` | `x`, `y`, `z` | `raw_x` に使用する `MagneticField` 軸 |
| `raw_x_sign` | `double` | `-1.0` | finite non-zero | `raw_x_axis` に掛ける符号 |
| `raw_z_axis` | `string` | `x` | `x`, `y`, `z` | `raw_z` に使用する `MagneticField` 軸 |
| `raw_z_sign` | `double` | `1.0` | finite non-zero | `raw_z_axis` に掛ける符号 |

### 7.1 Parameter Requirements

| ID | Requirement |
| --- | --- |
| PR-001 | `mag_topic` が空文字の場合、起動時に validation error とすること。 |
| PR-002 | `center_x`, `center_z`, `zero_heading_deg` は source code 固定値ではなく ROS parameter から設定できること。 |
| PR-003 | `publish_rate_hz <= 0.0` の場合、起動時に validation error とすること。 |
| PR-004 | `frame_id` が空文字の場合、起動時に validation error とすること。 |
| PR-005 | `center_x`, `center_z`, `soft_iron_matrix_00`, `soft_iron_matrix_01`, `soft_iron_matrix_10`, `soft_iron_matrix_11`, `zero_heading_deg`, `magnetic_field_scale`, `diagnostic_log_interval_sec`, `raw_x_sign`, `raw_z_sign` が finite value でない場合、起動時に validation error とすること。 |
| PR-006 | `diagnostic_log_interval_sec < 0.0` の場合、起動時に validation error とすること。 |
| PR-007 | `raw_x_axis`, `raw_z_axis` が `x`, `y`, `z` 以外の場合、起動時に validation error とすること。 |
| PR-008 | `raw_x_sign`, `raw_z_sign` が `0.0` の場合、起動時に validation error とすること。 |
| PR-009 | 起動時 log に `mag_topic`, `center_x`, `center_z`, soft-iron 補正行列, `zero_heading_deg`, `publish_rate_hz`, `frame_id`, `invert_yaw`, `magnetic_field_scale`, `diagnostic_log_interval_sec`, raw axis mapping を出力すること。 |
| PR-010 | parameter は launch 引数または YAML から設定できること。 |

### 7.2 YAML Example

```yaml
zed_heading_publisher:
  ros__parameters:
    mag_topic: /zed2i/zed_node/imu/mag
    center_x: -0.629279
    center_z: -0.874388
    zero_heading_deg: 39.7
    publish_rate_hz: 20.0
    frame_id: zed2i_mag
    invert_yaw: false
    magnetic_field_scale: 1000000.0
    diagnostic_log_interval_sec: 1.0
    raw_x_axis: "y"
    raw_x_sign: -1.0
    raw_z_axis: "x"
    raw_z_sign: 1.0
```

## 8. 角度定義

### 8.1 Calibration Tool

ZED wrapper の magnetometer topic に合わせて `center_x`, `center_z`, `zero_heading_deg` と soft-iron 補正行列を取り直すため、以下の実行ファイルを提供する。

```bash
ros2 run zed_heading_publisher calibrate_zed_heading
```

default では ZED VIO odometry topic を使用し、キャリブレーション開始時の VIO yaw をロボット実 yaw = 0 deg として扱う。キャリブレーション中はロボットをその場でゆっくり1回転させる。VIO の累積回転量が `target_rotation_deg` 以上になり、かつ角度 bin の coverage が `min_bin_coverage_ratio` 以上になった時点で自動終了する。

MAG の最初の有効 sample を yaw 0 deg 基準 sample として保持し、soft-iron 補正値を求めたあと、その sample から `zero_heading_deg` を計算する。したがって終了直前に yaw 0 deg へ戻す必要はない。

VIO を使わない従来の時間ベース収集を使う場合は以下のように指定する。

```bash
ros2 run zed_heading_publisher calibrate_zed_heading --ros-args \
  -p use_vio_turn_detection:=false \
  -p duration_sec:=30.0
```

VIO mode では `max_duration_sec` を安全 timeout として使用する。timeout 時点でも sample があれば計算結果を出力するが、log と diagnostics で `finish_reason` を確認すること。

サンプル数が `min_samples` 未満の場合は警告を出すが、1サンプル以上あれば計算結果は出力する。

`sample_csv_path` を指定すると、収集した `raw_x`, `raw_z` sample を CSV として保存する。

```bash
ros2 run zed_heading_publisher calibrate_zed_heading --ros-args \
  -p sample_csv_path:=/tmp/zed_heading_samples.csv
```

`input_csv_path` を指定すると、ROS topic から収集せず、CSV の `raw_x`, `raw_z` column から同じ補正値を再計算する。この場合、CSV の先頭 sample を yaw 0 deg 基準 sample として扱う。

```bash
ros2 run zed_heading_publisher calibrate_zed_heading --ros-args \
  -p input_csv_path:=/tmp/zed_heading_samples.csv
```

終了時に `zed_heading_publisher.yaml` へ転記できる形式で以下を標準出力へ表示する。

```yaml
zed_heading_publisher:
  ros__parameters:
    center_x: ...
    center_z: ...
    soft_iron_matrix_00: ...
    soft_iron_matrix_01: ...
    soft_iron_matrix_10: ...
    soft_iron_matrix_11: ...
    zero_heading_deg: ...
```

### 8.2 VIO/MAG Heading Comparison Tool

ZED wrapper のVIO yawと、magnetometer由来の `/zed/heading` yawを比較するため、以下の実行ファイルを提供する。

```bash
ros2 run zed_heading_publisher compare_vio_heading --ros-args \
  -p heading_topic:=/zed/heading \
  -p odom_topic:=/zed2i/zed_node/odom \
  -p csv_path:=/tmp/zed_vio_mag_compare.csv
```

このツールは `zed_interfaces/msg/ZedHeading` と `nav_msgs/msg/Odometry` をsubscribeし、初回に受信できた両yawをゼロ基準として、以後の相対yawを比較する。

CSVには以下を出力する。

```text
time_sec
mag_stamp_sec
vio_stamp_sec
mag_yaw_deg
vio_yaw_deg
mag_relative_yaw_deg
vio_relative_yaw_deg
yaw_diff_deg
mag_age_sec
vio_age_sec
```

### 8.3 Coordinate Selection

`sensor_msgs/msg/MagneticField` から、旧SDK直読み実装で使っていた `raw_x`（ロボット右方向相当）と `raw_z`（ロボット前方向相当）を作る。

ZED wrapper の ROS topic では、水平面は `x,y` 軸として扱われるため、default は以下とする。

```text
raw_x = -msg.magnetic_field.y * magnetic_field_scale
raw_z =  msg.magnetic_field.x * magnetic_field_scale
```

`sensor_msgs/msg/MagneticField` は ROS 標準では Tesla 単位である。既存の `center_x`, `center_z` は microtesla 相当の値として扱っているため、default では `magnetic_field_scale: 1000000.0` を適用する。

旧実装相当の `msg.magnetic_field.x` / `msg.magnetic_field.z` を確認したい場合は、以下の parameter に変更する。

```text
raw_x_axis = x
raw_x_sign = 1.0
raw_z_axis = z
raw_z_sign = 1.0
```

### 8.4 Hard-Iron Correction

360 度回転データから求めた磁場中心を引き、soft-iron 補正行列を適用する。

```text
shifted_x = raw_x - center_x
shifted_z = raw_z - center_z

corrected_x = soft_iron_matrix_00 * shifted_x + soft_iron_matrix_01 * shifted_z
corrected_z = soft_iron_matrix_10 * shifted_x + soft_iron_matrix_11 * shifted_z
```

soft-iron 補正行列の default は単位行列であり、未設定時は従来の hard-iron 補正のみと同じ挙動になる。

### 8.5 Magnetic Heading

補正後の X-Z 平面から磁気角を算出する。

```text
magnetic_heading_deg = atan2(corrected_x, corrected_z) [deg]
magnetic_heading_deg = normalize_to_180(magnetic_heading_deg)
```

正規化範囲:

```text
[-180, 180)
```

### 8.6 Robot Yaw

ロボット実 yaw = 0 deg のときに計測した磁気角 `zero_heading_deg` を引く。

```text
robot_yaw_deg = magnetic_heading_deg - zero_heading_deg
robot_yaw_deg = normalize_to_180(robot_yaw_deg)
```

`invert_yaw == true` の場合は、正規化前に符号を反転する。

```text
robot_yaw_deg = -robot_yaw_deg
robot_yaw_deg = normalize_to_180(robot_yaw_deg)
```

rad 値は以下で算出する。

```text
robot_yaw_rad = radians(robot_yaw_deg)
```

### 8.7 Positive Direction

ロボットが左旋回したときに `robot_yaw_deg` と `robot_yaw_rad` が増加する向きを正方向とする。

現時点の実機確認では、ZED2i magnetometer の `atan2(corrected_x, corrected_z)` に基づく角度は左旋回で増加する。そのため `invert_yaw` の default は `false` とする。

## 9. Angle Normalization

角度正規化関数は以下の仕様とする。

```text
normalize_to_180(angle_deg) = (angle_deg + 180.0) % 360.0 - 180.0
```

期待値:

| Input [deg] | Output [deg] |
| --- | --- |
| `0.0` | `0.0` |
| `180.0` | `-180.0` |
| `181.0` | `-179.0` |
| `359.0` | `-1.0` |
| `-181.0` | `179.0` |
| `-360.0` | `0.0` |

## 10. Control Flow

本ノードは明示的な state machine を持たない。magnetometer callback により sensor data 受信、補正、publish を行う。

```text
STARTUP
  -> declare parameters
  -> validate parameters
  -> create publisher
  -> create magnetometer subscription
  -> MAG_CALLBACK

MAG_CALLBACK
  -> receive sensor_msgs/msg/MagneticField
  -> apply publish_rate_hz throttle
  -> extract raw_x and raw_z
  -> apply hard-iron correction
  -> calculate magnetic_heading_deg
  -> calculate robot_yaw_deg
  -> calculate robot_yaw_rad
  -> publish /zed/heading
```

### 10.1 Startup Requirements

| ID | Requirement |
| --- | --- |
| CF-001 | node 起動時に全 parameter を宣言、取得、validation すること。 |
| CF-002 | `mag_topic` の `sensor_msgs/msg/MagneticField` subscription を作成すること。 |
| CF-003 | magnetometer subscription は sensor data QoS を使用すること。 |
| CF-004 | ZED2i camera device を直接 open しないこと。 |

### 10.2 Timer Loop Requirements

| ID | Requirement |
| --- | --- |
| CF-010 | magnetometer message を受信した場合のみ処理すること。 |
| CF-011 | publish interval は `1.0 / publish_rate_hz` 以上とすること。 |
| CF-012 | raw axis mapping parameter に従って `msg.magnetic_field` から `raw_x`, `raw_z` を抽出すること。 |
| CF-013 | publish message の `header.stamp` は publish 時の ROS clock とすること。 |
| CF-014 | publish message の `header.frame_id` は `frame_id` parameter とすること。 |
| CF-015 | publish 成功時の `valid` は `true` とすること。 |

## 11. QoS

`/zed/heading` publish は初期実装では以下を使用する。

```text
history: keep last
depth: 10
reliability: reliable
```

ZED wrapper 由来の `/zed2i/zed_node/imu/mag` subscription は sensor data QoS を使用する。

## 12. Failure Cases

| ID | Failure Case | Required Behavior |
| --- | --- | --- |
| FC-001 | parameter validation error | 起動失敗とする |
| FC-002 | `mag_topic` が未受信 | `/zed/heading` を publish しない |
| FC-003 | raw 磁場値が finite value でない | warn log を出し、その message では publish しない |
| FC-004 | input rate が `publish_rate_hz` より高い | publish を throttle する |

warn log は必要に応じて throttle し、センサ取得失敗が連続した場合でも log が過剰に増えないようにする。

## 13. Safety Requirements

| ID | Requirement |
| --- | --- |
| SR-001 | magnetometer data 未受信時に古い yaw を再 publish しないこと。 |
| SR-002 | raw 磁場値が不正な場合に推定値を捏造して publish しないこと。 |
| SR-003 | 本ノードは速度指令や motor command を publish しないこと。 |
| SR-004 | `center_x`, `center_z`, `zero_heading_deg` の使用値を起動 log で追跡できること。 |
| SR-005 | yaw 正方向は左旋回で増加する向きとし、後段の `angular.z` 正方向と整合させること。 |
| SR-006 | 本ノードは ZED camera device を直接 open しないこと。 |

## 14. 実機確認項目

| ID | Test | Expected Result |
| --- | --- | --- |
| TT-001 | ロボット実 yaw = 0 deg 方向に置いて起動する | `robot_yaw_deg` が `0 deg` 付近になる |
| TT-002 | ロボットを左にゆっくり旋回する | `robot_yaw_deg` が増加する |
| TT-003 | ロボットを右にゆっくり旋回する | `robot_yaw_deg` が減少する |
| TT-004 | `+179 deg` 付近からさらに左旋回する | `robot_yaw_deg` が `-180 deg` 付近へ連続的に折り返す |
| TT-005 | `/zed/heading` を rosbag 記録する | raw, corrected, yaw が同一 timestamp の message として記録される |
| TT-006 | `center_x`, `center_z`, `zero_heading_deg` を YAML で変更して起動する | 起動 log と publish 値に parameter 変更が反映される |
| TT-007 | `/zed2i/zed_node/imu/mag` を停止する | `/zed/heading` が古い値を再 publish しない |

## 15. Future Extensions

将来、magnetometer data と VIO/IMU を統合した sensor fusion を行う場合は、`zed_heading_publisher` ではなく別の fusion node を追加する。

## 校正結果の自動保存

`ros2 run zed_heading_publisher calibrate_zed_heading` は校正完了時に
`config/zed_heading_publisher.yaml` の校正値・入力トピック・計測軸・磁場スケールを自動更新する。
配信周期、frame_id、invert_yawなど、校正に関係しない設定値は保持する。
YAMLは再出力されるためコメントや引用符などの書式は保持しない。

ソース実行・symlink-installではソース側のconfigを更新する。
通常インストールではパッケージのshare/configを更新するため、変更をソースに残す場合は保存先を明示する。
保存先は既存の正しいYAMLである必要があり、書き込み失敗はエラーログに出力する。
ファイルは一時ファイルから置換し、既存YAMLのシンボリックリンクは維持する。

```bash
ros2 run zed_heading_publisher calibrate_zed_heading --ros-args -p output_config_path:=/absolute/path/to/zed_heading_publisher.yaml
# 表示だけ行い、保存しない
ros2 run zed_heading_publisher calibrate_zed_heading --ros-args -p save_config:=false
```

CSVからの再校正も同じ保存仕様。サンプル数がmin_samples未満、計算失敗、
VIOモードで旋回完了前のmax_duration_sec到達の場合は上書きしない。
起動済みの方位配信ノードには自動反映されないため、保存後に再起動する。
