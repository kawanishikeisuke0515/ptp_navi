# UWB OptiTrack Logger 仕様

## 1. 目的

`uwb_optitrack_logger` は、UWB 測距データと OptiTrack の姿勢データを同一 CSV ファイルへ時系列ログとして保存するノードである。

実験後に UWB 距離と OptiTrack 真値を比較し、UWB による位置推定、キャリブレーション、測距安定性評価に利用できるデータを残すことを目的とする。

## 2. 対象範囲

本仕様の対象は、既に ROS 2 topic として publish されている以下のデータを subscribe し、一定周期で CSV に保存する処理である。

- UWB 測距データ
- OptiTrack pose

本ノードはロギングと、3 アンカー距離からの簡易 2D 三辺測量を担当する。OptiTrack 座標系との座標変換、フィルタリング、制御入力生成は対象外とする。

## 3. ノード仕様

### 3.1 Node

```text
node name: uwb_optitrack_logger
executable: uwb_optitrack_logger
package: uwb_position_publisher
```

### 3.2 Responsibilities

`uwb_optitrack_logger` は以下を担当する。

1. パラメータの宣言と取得
2. 出力ディレクトリ作成
3. CSV ファイル作成と header 書き込み
4. `/uwb/distances` の subscribe
5. OptiTrack pose topic の subscribe
6. 各 topic の最新値保持
7. 有効な 3 アンカー距離から UWB 推定位置 `x, y` を計算
8. 指定周期で最新値を 1 行の CSV として保存
9. 一定行数ごとの flush
10. ノード終了時の flush と close

## 4. 入力仕様

### 4.1 UWB Topic

```text
default topic: /uwb/distances
type: uwb_interfaces/msg/UwbDistances
```

message:

```text
std_msgs/Header header
uint32 device_time_ms

float32 anchor_1_distance_m
float32 anchor_2_distance_m
float32 anchor_3_distance_m

bool anchor_1_valid
bool anchor_2_valid
bool anchor_3_valid

string raw_line
```

### 4.2 OptiTrack Topic

```text
default topic: /vrpn_mocap/RigidBody_1/pose
type: geometry_msgs/msg/PoseStamped
```

message 内の以下をログ対象とする。

```text
header.stamp
pose.position.x
pose.position.y
pose.position.z
pose.orientation.x
pose.orientation.y
pose.orientation.z
pose.orientation.w
```

## 5. パラメータ仕様

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `output_dir` | string | `/tmp/uwb_optitrack_logs` | CSV 保存先ディレクトリ |
| `log_rate` | double | `20.0` | CSV 書き込み周期 [Hz] |
| `flush_every_rows` | int | `20` | CSV flush を行う行数間隔 |
| `uwb_topic` | string | `/uwb/distances` | subscribe する UWB topic |
| `optitrack_pose_topic` | string | `/vrpn_mocap/RigidBody_1/pose` | subscribe する OptiTrack pose topic |
| `anchor_1_x` | double | `0.0` | アンカー 1 の x 座標 [m] |
| `anchor_1_y` | double | `0.0` | アンカー 1 の y 座標 [m] |
| `anchor_2_x` | double | `1.0` | アンカー 2 の x 座標 [m] |
| `anchor_2_y` | double | `0.0` | アンカー 2 の y 座標 [m] |
| `anchor_3_x` | double | `0.0` | アンカー 3 の x 座標 [m] |
| `anchor_3_y` | double | `1.0` | アンカー 3 の y 座標 [m] |
| `min_anchor_determinant` | double | `1.0e-9` | 3 点測位の行列特異判定しきい値 |

### 5.1 Parameter Validation

起動時に以下を検証する。

- `log_rate > 0.0`
- `flush_every_rows > 0`
- `output_dir` が空文字列ではない
- `uwb_topic` が空文字列ではない
- `optitrack_pose_topic` が空文字列ではない
- `min_anchor_determinant > 0.0`

条件を満たさない場合は `ValueError` を送出し、ノードを起動しない。

## 6. 出力仕様

### 6.1 CSV File

CSV ファイル名は起動時刻を含める。

```text
uwb_optitrack_log_<timestamp>.csv
```

timestamp はローカル時刻で以下の形式とする。

```text
YYYYMMDD_HHMMSS
```

例:

```text
uwb_optitrack_log_20260728_153012.csv
```

### 6.2 CSV Columns

初期実装では以下の列を出力する。

```text
elapsed_sec,
uwb_stamp_sec,
uwb_device_time_ms,
anchor_1_distance_m,
anchor_2_distance_m,
anchor_3_distance_m,
anchor_1_valid,
anchor_2_valid,
anchor_3_valid,
uwb_raw_line,
uwb_position_valid,
uwb_estimated_x,
uwb_estimated_y,
optitrack_stamp_sec,
optitrack_x,
optitrack_y,
optitrack_z,
optitrack_qx,
optitrack_qy,
optitrack_qz,
optitrack_qw
```

### 6.3 Column Definitions

| Column | Description |
| --- | --- |
| `elapsed_sec` | logger 起動からの経過時間 [s] |
| `uwb_stamp_sec` | `UwbDistances.header.stamp` を秒に変換した値 |
| `uwb_device_time_ms` | UWB デバイス側時刻 [ms] |
| `anchor_1_distance_m` | アンカー 1 との距離 [m] |
| `anchor_2_distance_m` | アンカー 2 との距離 [m] |
| `anchor_3_distance_m` | アンカー 3 との距離 [m] |
| `anchor_1_valid` | アンカー 1 測距有効フラグ |
| `anchor_2_valid` | アンカー 2 測距有効フラグ |
| `anchor_3_valid` | アンカー 3 測距有効フラグ |
| `uwb_raw_line` | UWB publisher が保持する入力 raw line |
| `uwb_position_valid` | UWB 3 点測位結果の有効フラグ |
| `uwb_estimated_x` | 3 アンカー距離から推定した UWB x 座標 [m] |
| `uwb_estimated_y` | 3 アンカー距離から推定した UWB y 座標 [m] |
| `optitrack_stamp_sec` | `PoseStamped.header.stamp` を秒に変換した値 |
| `optitrack_x` | OptiTrack position x |
| `optitrack_y` | OptiTrack position y |
| `optitrack_z` | OptiTrack position z |
| `optitrack_qx` | OptiTrack orientation quaternion x |
| `optitrack_qy` | OptiTrack orientation quaternion y |
| `optitrack_qz` | OptiTrack orientation quaternion z |
| `optitrack_qw` | OptiTrack orientation quaternion w |

## 7. UWB 2D 位置推定

### 7.1 Anchor Coordinate

3 点測位に使用するアンカー座標は、logger 起動時のパラメータで指定する。

```text
anchor_1: (anchor_1_x, anchor_1_y)
anchor_2: (anchor_2_x, anchor_2_y)
anchor_3: (anchor_3_x, anchor_3_y)
```

座標系は UWB 実験系の任意の 2D 座標系とする。OptiTrack 座標系と一致している必要はないが、比較を容易にするため、実験時には OptiTrack 座標系または後処理で変換しやすい座標系に合わせることを推奨する。

### 7.2 Trilateration Formula

アンカー座標を以下とする。

```text
A1 = (x1, y1)
A2 = (x2, y2)
A3 = (x3, y3)
```

各アンカーとの距離を以下とする。

```text
r1, r2, r3
```

推定位置 `P = (x, y)` は、円の方程式を差分で線形化して求める。

```text
2(x2 - x1)x + 2(y2 - y1)y = r1^2 - r2^2 - x1^2 + x2^2 - y1^2 + y2^2
2(x3 - x1)x + 2(y3 - y1)y = r1^2 - r3^2 - x1^2 + x3^2 - y1^2 + y3^2
```

2x2 行列を解き、`uwb_estimated_x`, `uwb_estimated_y` として CSV に保存する。

### 7.3 Validity

以下をすべて満たす場合のみ、UWB 推定位置を有効とする。

- `anchor_1_valid == true`
- `anchor_2_valid == true`
- `anchor_3_valid == true`
- `anchor_1_distance_m`, `anchor_2_distance_m`, `anchor_3_distance_m` が有限値である
- アンカー 3 点が同一直線上ではない

アンカー 3 点の行列 determinant の絶対値が `min_anchor_determinant` 以下の場合、同一直線上または数値的に不安定と判断する。

推定できない場合は以下のように出力する。

```text
uwb_position_valid: false
uwb_estimated_x:
uwb_estimated_y:
```

推定できる場合は以下のように出力する。

```text
uwb_position_valid: true
uwb_estimated_x: calculated x
uwb_estimated_y: calculated y
```

## 8. ロギング方式

### 8.1 Latest Sample Hold

各 subscribe callback では、受信した最新 message を内部変数に保持する。

CSV 書き込み timer callback では、timer 実行時点で保持されている最新 UWB message、UWB 推定位置、最新 OptiTrack pose を同一行に書き込む。

この方式は厳密な時刻同期ではなく、後処理で時刻列を参照して比較するための簡易同期ログである。

### 8.2 Missing Data

ノード起動直後など、まだ該当 topic を受信していない場合、その topic に対応する列は空文字列として出力する。

例:

```text
elapsed_sec,uwb_stamp_sec,...,optitrack_stamp_sec,optitrack_x,...
0.0500,1785279001.123456,...,,,,,,,
```

### 8.3 Numeric Format

数値列は原則として小数 6 桁で出力する。

```text
1.234567
```

`elapsed_sec` は小数 4 桁で出力する。

```text
12.3456
```

bool 列は Python の bool 表現ではなく、CSV で扱いやすいように以下の文字列で出力する。

```text
true
false
```

未受信の場合は空文字列とする。

## 9. QoS

### 9.1 UWB Topic

初期実装では default QoS を使用する。

```text
history: keep last
depth: 10
reliability: reliable
```

`uwb_distance_publisher` の publish QoS と合わせる。

### 9.2 OptiTrack Topic

OptiTrack / VRPN 系 topic は best effort で publish される場合があるため、初期実装では best effort の subscribe QoS を使用する。

```text
history: keep last
depth: 10
reliability: best effort
```

## 10. 起動例

```bash
ros2 run uwb_position_publisher uwb_optitrack_logger --ros-args \
  -p output_dir:=/tmp/uwb_optitrack_logs \
  -p log_rate:=20.0 \
  -p uwb_topic:=/uwb/distances \
  -p optitrack_pose_topic:=/vrpn_mocap/RigidBody_1/pose \
  -p anchor_1_x:=0.0 \
  -p anchor_1_y:=0.0 \
  -p anchor_2_x:=1.0 \
  -p anchor_2_y:=0.0 \
  -p anchor_3_x:=0.0 \
  -p anchor_3_y:=1.0
```

## 11. 確認コマンド

UWB topic:

```bash
ros2 topic echo /uwb/distances
```

OptiTrack topic:

```bash
ros2 topic echo /vrpn_mocap/RigidBody_1/pose
```

CSV:

```bash
ls /tmp/uwb_optitrack_logs
```

## 12. 異常系

### 12.1 Output Directory Creation Failure

`output_dir` の作成に失敗した場合、ノードは例外で終了する。

実験開始前に保存先の permission と空き容量を確認する。

### 12.2 Topic 未受信

UWB または OptiTrack のどちらかが未受信でも、ノードは終了しない。

未受信 topic の列は空文字列として出力し、受信済み topic の値は通常通り保存する。

### 12.3 NaN Distance

`UwbDistances` 内の距離が `NaN` の場合、CSV には `nan` として保存する。

有効性の判断には `anchor_*_valid` 列を使用する。

### 12.4 UWB Position Invalid

3 アンカー距離がそろわない場合、距離が `NaN` の場合、またはアンカー配置が不正な場合でも、ノードは終了しない。

この場合、`uwb_position_valid` を `false` とし、`uwb_estimated_x`, `uwb_estimated_y` は空文字列として出力する。

## 13. 将来拡張

必要に応じて以下を追加する。

- UWB 距離と OptiTrack pose の近傍時刻同期
- OptiTrack yaw の CSV 出力
- ROS bag 出力との併用
- 実験条件を保存する metadata CSV または YAML


### タグとアンカーの高さ差の設定

`anchor_1_height_difference_m`、`anchor_2_height_difference_m`、
`anchor_3_height_difference_m` に、各アンカーに対する
「タグの高さ − アンカーの高さ」をメートルで設定する。
既定値はすべて `0.0`（従来どおり）。値は有限値に限り、負値も指定できる。
例えばタグが床から 0.5 m、アンカーが 1.5 m の場合は `-1.0` とする。
3 台とも同じ高さなら、3 パラメータに同じ値を設定する。

位置配信ノードは `config/uwb_position_publisher.yaml` を編集して起動する。
OptiTrack ロガーにも同名の ROS パラメータを設定する
（例: `-p anchor_1_height_difference_m:=-1.0`。残り 2 台も設定）。
設定は起動時に読み込むため、設置高さを変えた場合は設定変更後にノードを再起動する。

各測距値 `d_i` と高さ差 `h_i` から水平距離の二乗
`r_i² = d_i² - h_i²` を求め、2D 三辺測量に使う。
`d_i < |h_i|` の測距は位置無効とする。等しい場合の水平距離 0 は許容する。
`/uwb/distances` と CSV の測距列は元の空間距離を保持し、推定出力は従来と同じ x/y 座標となる。
高さ自体の推定や、動作中に変化する高さの自動追従は行わない。

3 台のアンカーが同じ高さの場合、高さ補正項は方程式の差分で相殺されるため、
有効な測距に対する x/y は補正前と一致する。
アンカーごとに高さが異なる場合は、この補正によって x/y が変わる。
