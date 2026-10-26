# UWB Distance Publisher 設計仕様

## 1. 概要

`uwb_position_publisher` は、Arduino から USB シリアルで送信される UWB 測距 CSV を読み取り、ROS 2 トピック `/uwb/distances` に publish し、別ノードで 3 アンカー距離から `/uwb/position` を publish するパッケージである。

`uwb_distance_publisher` は測距データの取り込み、入力検証、単位変換、無効値判定、再接続処理を担当する。`uwb_position_publisher` は `/uwb/distances` を subscribe し、アンカー座標を用いて 2D 位置推定を担当する。

## 2. 設計方針

- シリアル入力と距離 publish の責務を `uwb_distance_publisher` に閉じる。
- 位置推定と位置 publish の責務を `uwb_position_publisher` に閉じる。
- 入力 CSV の不正行は publish せず破棄し、warn ログで原因を追跡できるようにする。
- 距離値の単位は ROS 側で SI 単位系に統一し、cm から m に変換する。
- 測距失敗は `NaN` と valid flag の組で表現する。
- シリアル切断時もノードを終了させず、一定間隔で再接続する。
- 位置推定ノードが raw data と validity を判断できる情報を `/uwb/distances` に保持する。

## 3. パッケージ構成

### 3.1 実行パッケージ

```text
uwb_position_publisher
```

想定する主要ファイル:

```text
uwb_position_publisher/
  package.xml
  setup.py
  setup.cfg
  resource/uwb_position_publisher
  uwb_position_publisher/
    __init__.py
    uwb_distance_publisher.py
    uwb_position_publisher.py
    uwb_position_logger.py
  launch/
    uwb_distance_publisher.launch.py
    uwb_position_logger.launch.py
  config/
    uwb_distance_publisher.yaml
    uwb_position_publisher.yaml
    uwb_position_logger.yaml
  doc/
    requirements_ja.md
    design_ja.md
```

### 3.2 メッセージパッケージ

カスタムメッセージは別パッケージに分離する。

```text
uwb_interfaces
```

想定する主要ファイル:

```text
uwb_interfaces/
  package.xml
  CMakeLists.txt
  msg/
    UwbDistances.msg
    UwbPosition.msg
```

## 4. ノード設計

### 4.1 Node

```text
node name: uwb_distance_publisher
executable: uwb_distance_publisher

node name: uwb_position_publisher
executable: uwb_position_publisher

node name: uwb_position_logger
executable: uwb_position_logger
```

### 4.2 Responsibilities

`uwb_distance_publisher` は以下を担当する。

1. パラメータの宣言と取得
2. シリアルポートの open
3. シリアル行読み取り
4. CSV の正規化と parse
5. 距離値の妥当性判定
6. cm から m への単位変換
7. `UwbDistances` メッセージ生成
8. `/uwb/distances` への publish
9. シリアル異常時の close と reconnect

`uwb_position_publisher` は以下を担当する。

1. `/uwb/distances` の subscribe
2. アンカー座標 parameter の宣言と取得
3. 3 アンカー距離の validity 判定
4. 2D trilateration による `x_m`, `y_m` 推定
5. `UwbPosition` メッセージ生成
6. `/uwb/position` への publish

`uwb_position_logger` は以下を担当する。

1. `/uwb/position` の subscribe
2. CSV ファイルの作成
3. 受信した `UwbPosition` 1 message ごとの CSV 行保存
4. 一定行数ごとの flush

## 5. パラメータ設計

### 5.1 `uwb_distance_publisher`

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `serial_port` | string | `/dev/ttyACM1` | Arduino のシリアルデバイスパス |
| `baudrate` | int | `115200` | シリアル通信速度 |
| `read_timeout` | double | `0.1` | 1 回の read timeout [s] |
| `reconnect_interval` | double | `1.0` | 再接続試行間隔 [s] |
| `invalid_distance_cm` | int | `65535` | 測距失敗を表す距離値 [cm] |
| `frame_id` | string | `uwb` | publish する header の frame id |
| `timer_period` | double | `0.01` | シリアル読み取り周期 [s] |

### 5.2 `uwb_position_publisher`

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `uwb_distances_topic` | string | `/uwb/distances` | subscribe する UWB 距離 topic |
| `uwb_position_topic` | string | `/uwb/position` | publish する UWB 位置 topic |
| `anchor_1_x` | double | `0.0` | アンカー 1 の x 座標 [m] |
| `anchor_1_y` | double | `0.0` | アンカー 1 の y 座標 [m] |
| `anchor_2_x` | double | `1.0` | アンカー 2 の x 座標 [m] |
| `anchor_2_y` | double | `0.0` | アンカー 2 の y 座標 [m] |
| `anchor_3_x` | double | `0.0` | アンカー 3 の x 座標 [m] |
| `anchor_3_y` | double | `1.0` | アンカー 3 の y 座標 [m] |
| `min_anchor_determinant` | double | `1.0e-9` | アンカー配置の特異判定しきい値 |
| `moving_average_window` | int | `5` | `/uwb/position` の `x_m`, `y_m` に適用する移動平均サンプル数 |
| `max_position_jump_m` | double | `1.0` | 前回フィルタ済み位置からこの距離 [m] を超えて飛んだ valid サンプルを移動平均に入れず、直前のフィルタ済み位置を保持する。`0.0` 以下で無効 |
| `filter_reset_on_invalid` | bool | `true` | invalid サンプル受信時に移動平均バッファをリセットする |

### 5.3 `uwb_position_logger`

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `output_dir` | string | `/tmp/uwb_position_logs` | CSV 保存先ディレクトリ |
| `uwb_position_topic` | string | `/uwb/position` | subscribe する UWB 位置 topic |
| `uwb_distances_topic` | string | `/uwb/distances` | subscribe する UWB 距離 topic |
| `optitrack_pose_topic` | string | `/vrpn_mocap/RigidBody_1/pose` | subscribe する OptiTrack pose topic |
| `flush_every_rows` | int | `20` | CSV を flush する行数間隔 |

実運用では USB 接続順による `/dev/ttyACM*` の番号変化を避けるため、`serial_port` に `/dev/serial/by-id/...` を指定することを推奨する。

安定デバイス名は以下で確認する。

```bash
ls -l /dev/serial/by-id/
```

## 6. 入力設計

### 6.1 シリアル設定

Python 実装では `pyserial` を使用する。

```text
port: serial_port parameter
baudrate: baudrate parameter
timeout: read_timeout parameter
```

### 6.2 入力フォーマット

1 行 1 サンプルの CSV とする。

```text
device_time_ms,anchor_1_distance_cm,anchor_2_distance_cm,anchor_3_distance_cm
```

括弧付き入力も許容する。

```text
(29907,46,156,65535)
```

### 6.3 正規化

read した 1 行に対して以下を順に適用する。

1. bytes の場合は UTF-8 で decode する。
2. 前後空白、改行、復帰文字を除去する。
3. 先頭と末尾が `(`, `)` の場合は外側の括弧を除去する。
4. comma で分割する。
5. 各列の前後空白を除去する。

空行は warn 対象にせず、publish せず無視する。

## 7. Parse 設計

### 7.1 データ構造

内部処理では parse 結果を以下の情報として扱う。

```text
device_time_ms: int
distance_cm: [int, int, int]
raw_line: string
```

### 7.2 Parse 条件

publish 可能な入力条件:

- CSV 分割後の列数が 4 である。
- 1 列目が 0 以上の整数として解釈できる。
- 2 から 4 列目が整数として解釈できる。

列数不一致、数値変換失敗、負の `device_time_ms` は不正行として破棄する。

### 7.3 距離値の扱い

距離値はアンカーごとに独立して判定する。

```text
valid = distance_cm >= 0 and distance_cm != invalid_distance_cm
```

有効距離:

```text
distance_m = distance_cm / 100.0
valid = true
```

無効距離:

```text
distance_m = NaN
valid = false
```

`invalid_distance_cm` はデフォルト `65535` とする。UWB 側で値が入っていない場合に `65535` が出力されるため、この値は測距失敗として扱う。

0 未満の距離値も無効として扱う。これにより `-1` などの負値が入力された場合も測距失敗として安全に処理できる。

## 8. 出力設計

### 8.1 Topic

```text
topic: /uwb/distances
type: uwb_interfaces/msg/UwbDistances
publisher node: uwb_distance_publisher

topic: /uwb/position
type: uwb_interfaces/msg/UwbPosition
publisher node: uwb_position_publisher
```

### 8.2 QoS

初期実装では以下を使用する。

```text
history: keep last
depth: 10
reliability: reliable
```

高頻度で最新値のみが重要になった場合は、`depth: 1` または `best effort` への変更を検討する。

### 8.3 Message

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

`/uwb/position` message:

```text
std_msgs/Header header
uint32 device_time_ms

float32 x_m
float32 y_m

bool valid
```

3 アンカー距離がすべて有効で、アンカー配置が特異でない場合に `valid = true` とし、推定位置 `x_m`, `y_m` を publish する。

距離が無効、または 3 点測位を解けない場合は `valid = false` とし、`x_m`, `y_m` は `NaN` とする。

### 8.4 Position Log CSV

`uwb_position_logger` は `/uwb/position` を 1 message 受信するたびに、最新の `/uwb/distances` と最新の OptiTrack pose を合わせて 1 行保存する。`/uwb/distances` または OptiTrack pose が未受信の場合、該当列は空文字列とする。

```text
uwb_position_log_<timestamp>.csv
```

CSV columns:

```text
elapsed_sec,
position_stamp_sec,
device_time_ms,
x_m,
y_m,
valid,
distances_stamp_sec,
distances_device_time_ms,
anchor_1_distance_m,
anchor_2_distance_m,
anchor_3_distance_m,
anchor_1_valid,
anchor_2_valid,
anchor_3_valid,
raw_line,
optitrack_stamp_sec,
optitrack_x,
optitrack_y,
optitrack_z,
optitrack_qx,
optitrack_qy,
optitrack_qz,
optitrack_qw
```

### 8.5 Header

```text
header.stamp: ROS 2 ノードが行を受信し、publish メッセージを生成した時刻
header.frame_id: frame_id parameter
```

Arduino 側時刻は `header.stamp` には使用せず、`device_time_ms` に格納する。

## 9. 処理フロー

```text
start
  |
  v
declare parameters
  |
  v
create publisher /uwb/distances
  |
  v
open serial
  |
  v
timer callback
  |
  +-- serial is closed? ---------- yes --> try reconnect --> return
  |
  no
  |
  v
read one line
  |
  +-- no data? ------------------- yes --> return
  |
  no
  |
  v
normalize line
  |
  +-- empty line? ---------------- yes --> return
  |
  no
  |
  v
parse CSV
  |
  +-- parse error? --------------- yes --> warn log --> return
  |
  no
  |
  v
build UwbDistances message
  |
  v
publish
```

## 10. Timer 設計

シリアル読み取りは ROS 2 timer callback で実行する。

初期値:

```text
timer_period: 0.01 s
```

`read_timeout` は timer period より極端に長くしない。長い timeout は callback をブロックし、ノード停止や再接続の応答性を下げるためである。

将来的に入力周波数が高くなり timer callback で不足する場合は、シリアル読み取り専用 thread と queue に分離する。

## 11. 再接続設計

### 11.1 Open

ノード起動時に `serial_port` を open する。失敗した場合は error ログを出し、ノードは継続する。

### 11.2 Read Error

read 中に `SerialException` または OS レベルの入出力エラーが発生した場合:

1. error または warn ログを出す。
2. serial object を close する。
3. serial object を `None` にする。
4. 次回以降の timer callback で再接続を試みる。

### 11.3 Reconnect

最後の接続試行から `reconnect_interval` 秒以上経過した場合に open を再試行する。

再接続に成功した場合は info ログを出す。失敗が継続している間はログが過剰にならないよう、接続試行ごとに warn 以上を 1 回だけ出す。

## 12. Logging 設計

| Event | Level | 内容 |
| --- | --- | --- |
| ノード起動 | info | パラメータ概要 |
| シリアル open 成功 | info | port, baudrate |
| シリアル open 失敗 | warn/error | port, exception |
| 不正 CSV 列数 | warn | raw line, column count |
| 数値変換失敗 | warn | raw line |
| 負の `device_time_ms` | warn | raw line, value |
| シリアル read 失敗 | warn/error | exception |
| 再接続成功 | info | port |

正常な publish ごとの info ログは出さない。必要な場合は debug ログに限定する。

## 13. 異常系設計

### 13.1 空行

publish せず無視する。ログは出さない。

### 13.2 CSV 列数エラー

publish せず破棄する。warn ログを出す。

例:

```text
29907,46,156
```

### 13.3 数値変換エラー

publish せず破棄する。warn ログを出す。

例:

```text
29907,46,abc,-1
```

### 13.4 負の device time

publish せず破棄する。warn ログを出す。

例:

```text
-1,46,156,88
```

### 13.5 全アンカー無効

CSV として正しく parse できた場合は publish する。

例:

```text
29907,-1,-1,-1
```

出力:

```text
anchor_1_distance_m: NaN
anchor_2_distance_m: NaN
anchor_3_distance_m: NaN
anchor_1_valid: false
anchor_2_valid: false
anchor_3_valid: false
```

## 14. Launch / Config 設計

### 14.1 Config

```yaml
uwb_distance_publisher:
  ros__parameters:
    serial_port: "/dev/serial/by-id/usb-Arduino_Stella_DA6CCE29E172D7E7-if00"
    baudrate: 115200
    read_timeout: 0.1
    reconnect_interval: 1.0
    invalid_distance_cm: 65535
    frame_id: "uwb"
    timer_period: 0.01

uwb_position_publisher:
  ros__parameters:
    uwb_distances_topic: "/uwb/distances"
    uwb_position_topic: "/uwb/position"
    anchor_1_x: 0.0
    anchor_1_y: 0.0
    anchor_2_x: 1.0
    anchor_2_y: 0.0
    anchor_3_x: 0.0
    anchor_3_y: 1.0
    min_anchor_determinant: 1.0e-9
    moving_average_window: 5
    max_position_jump_m: 1.0
    filter_reset_on_invalid: true

uwb_position_logger:
  ros__parameters:
    output_dir: "/tmp/uwb_position_logs"
    uwb_position_topic: "/uwb/position"
    uwb_distances_topic: "/uwb/distances"
    optitrack_pose_topic: "/vrpn_mocap/RigidBody_1/pose"
    flush_every_rows: 20
```

### 14.2 Launch

launch file は distance / position それぞれの config yaml を読み込んで 2 ノードを起動する。

```bash
ros2 launch uwb_position_publisher uwb_distance_publisher.launch.py
```

直接実行する場合:

```bash
ros2 run uwb_position_publisher uwb_distance_publisher --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Arduino_Stella_DA6CCE29E172D7E7-if00 \
  -p baudrate:=115200

ros2 run uwb_position_publisher uwb_position_publisher --ros-args \
  -p uwb_distances_topic:=/uwb/distances \
  -p uwb_position_topic:=/uwb/position

ros2 launch uwb_position_publisher uwb_position_logger.launch.py
```

## 15. テスト設計

### 15.1 Unit Test

CSV parse と message 変換はシリアル実機なしでテストできるよう、関数として分離する。

テスト対象:

- 通常 CSV を parse できる。
- 括弧付き CSV を parse できる。
- 前後空白付き CSV を parse できる。
- cm から m に変換できる。
- `65535` が `NaN` と `valid=false` になる。
- `-1` が `NaN` と `valid=false` になる。
- 3 アンカーすべて無効でも parse 成功になる。
- 列数不一致を reject する。
- 数値変換失敗を reject する。
- 負の `device_time_ms` を reject する。

### 15.2 Integration Test

擬似シリアルまたは mock serial を使い、以下を確認する。

- 入力 1 行に対して `/uwb/distances` が 1 回 publish される。
- 不正行では publish されない。
- read exception 後に reconnect が呼ばれる。

### 15.3 Manual Test

起動:

```bash
ros2 run uwb_position_publisher uwb_distance_publisher --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Arduino_Stella_DA6CCE29E172D7E7-if00
```

確認:

```bash
ros2 topic echo /uwb/distances
```

期待値:

```text
CSV: 29907,46,156,65535

device_time_ms: 29907
anchor_1_distance_m: 0.46
anchor_2_distance_m: 1.56
anchor_3_distance_m: NaN
anchor_1_valid: true
anchor_2_valid: true
anchor_3_valid: false
raw_line: "29907,46,156,65535"
```

## 16. 実装メモ

### 16.1 推奨する関数分割

```text
normalize_line(line: str) -> str
parse_uwb_csv(line: str) -> ParsedUwbDistances
distance_cm_to_m(distance_cm: int, invalid_distance_cm: int) -> tuple[float, bool]
build_message(parsed: ParsedUwbDistances, stamp, frame_id: str) -> UwbDistances
try_open_serial() -> bool
close_serial() -> None
timer_callback() -> None
```

### 16.2 例外処理

parse error は内部例外または result object で表現する。timer callback では parse error を warn ログに変換し、ノードは継続する。

シリアル例外は reconnect 処理に流す。callback 外へ未処理例外を出してノードを落とさない。

### 16.3 NaN

Python 実装では `math.nan` を使用する。

```python
import math

distance_m = math.nan
```

## 17. 後続拡張

後段に位置推定ノードを追加する場合、以下の接続を想定する。

```text
/uwb/distances -> uwb_position_estimator -> /uwb/position
```

位置推定側では、各アンカーの座標、valid flag、距離値を使って三辺測量または最小二乗推定を行う。

本ノード側ではアンカー座標を持たず、測距値の publish に責務を限定する。


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
