# UWB Distance Publisher 要求仕様

## 1. 目的

Arduino から USB シリアルで出力される UWB 測距 CSV を ROS 2 トピックとして publish する。

UWB データは 3 個のアンカーとの距離情報であり、後段の位置推定ノードや制御ノードが利用できる形式に変換する。

## 2. 対象範囲

本仕様の対象は、Arduino のシリアル出力を読み取り、UWB 距離情報として ROS 2 に publish するノードである。

本仕様では、3 アンカー距離からロボット位置を推定する三辺測量処理は対象外とする。位置推定は後続ノードで扱う。

## 3. パッケージ構成

UWB 測距データは locomotion 制御本体とは責務が異なるため、新規パッケージとして分離する。

```text
uwb_position_publisher
```

カスタムメッセージを使用する場合は、メッセージ定義用パッケージを別途用意する。

```text
uwb_interfaces
```

## 4. 入力仕様

### 4.1 入力元

Arduino を Jetson に USB 接続し、シリアルポートから CSV 文字列を受信する。

デフォルト想定:

```text
port: /dev/serial/by-id/usb-Arduino_Stella_DA6CCE29E172D7E7-if00
baudrate: 115200
```

実運用では USB の接続順によるデバイス名変化を避けるため、`/dev/serial/by-id/...` を使用する。

### 4.2 CSV フォーマット

1 行につき 1 サンプルを出力する。

```text
device_time_ms,anchor_1_distance_cm,anchor_2_distance_cm,anchor_3_distance_cm
```

例:

```text
29907,46,156,65535
```

各列の意味:

```text
1列目: Arduino または UWB デバイス側の時刻 [ms]
2列目: アンカー1との距離 [cm]
3列目: アンカー2との距離 [cm]
4列目: アンカー3との距離 [cm]
```

入力行は括弧付きでも受け付ける。

```text
(29907,46,156,65535)
```

## 5. 出力仕様

### 5.1 Published Topic

```text
/uwb/distances
type: uwb_interfaces/msg/UwbDistances
```

### 5.2 メッセージ仕様

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

### 5.3 単位変換

Arduino からの距離値は cm とする。

ROS トピック上では SI 単位系に合わせ、m に変換して publish する。

```text
distance_m = distance_cm / 100.0
```

例:

```text
CSV: 29907,46,156,65535

device_time_ms: 29907
anchor_1_distance_m: 0.46
anchor_2_distance_m: 1.56
anchor_3_distance_m: NaN
anchor_1_valid: true
anchor_2_valid: true
anchor_3_valid: false
```

## 6. 無効値の扱い

距離値が負の場合、または距離値が `invalid_distance_cm` と一致する場合、そのアンカーの測距は失敗したものとして扱う。

デフォルトの無効値は、UWB 側で値なしを表す `65535` とする。

無効な距離は以下のように publish する。

```text
distance_m: NaN
valid: false
```

有効な距離は以下のように publish する。

```text
distance_m: distance_cm / 100.0
valid: true
```

## 7. ノード仕様

### 7.1 Node Name

```text
uwb_distance_publisher
```

### 7.2 Parameters

```text
serial_port: "/dev/serial/by-id/usb-Arduino_Stella_DA6CCE29E172D7E7-if00"
baudrate: 115200
read_timeout: 0.1
reconnect_interval: 1.0
invalid_distance_cm: 65535
frame_id: "uwb"
```

### 7.3 起動例

```bash
ros2 run uwb_position_publisher uwb_distance_publisher --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Arduino_Stella_DA6CCE29E172D7E7-if00 \
  -p baudrate:=115200
```

### 7.4 確認コマンド

```bash
ros2 topic echo /uwb/distances
```

## 8. 処理フロー

1. ノード起動時にシリアルポートを open する。
2. シリアルから 1 行ずつ読み取る。
3. 改行、前後空白、外側の括弧を除去する。
4. comma 区切りで 4 列に分割する。
5. 各列を数値に変換する。
6. 1 列目を `device_time_ms` に設定する。
7. 2 から 4 列目を cm から m に変換する。
8. 負の距離値、または `invalid_distance_cm` と一致する距離値は無効値として扱い、距離を `NaN`、valid flag を `false` にする。
9. `header.stamp` には ROS 2 ノードで受信した時刻を設定する。
10. `/uwb/distances` に publish する。

## 9. 異常系要求

### 9.1 CSV 列数エラー

4 列ではない行は publish せず破棄する。

該当行は warn ログに出力する。

### 9.2 数値変換エラー

数値に変換できない列を含む行は publish せず破棄する。

該当行は warn ログに出力する。

### 9.3 シリアル切断

シリアルポートが読めなくなった場合、ノードは終了せず、一定間隔で再接続を試みる。

再接続間隔は `reconnect_interval` パラメータで指定する。

### 9.4 全アンカー無効

3 つすべての距離が無効な場合でも、CSV として正しくパースできた場合は publish する。

この場合、後段ノードは valid flag を見てサンプルを採用するか判断する。

## 10. QoS

初期実装では以下を使用する。

```text
depth: 10
reliability: reliable
```

高頻度化して古いサンプルが不要になった場合は、depth を 1 にするか best effort を検討する。

## 11. 後続拡張

本ノードの後段に、3 アンカー距離から位置を推定するノードを追加できる。

```text
/uwb/distances -> uwb_position_estimator -> /uwb/position
```

想定される後続出力:

```text
/uwb/position
type: geometry_msgs/msg/PointStamped
```

または、推定状態や valid flag を含む専用メッセージを定義する。

## 12. 未確定事項

以下は実装前または実機確認時に確定する。

- 3 個のアンカーの実空間座標
- UWB デバイス時刻のカウンタ周期とオーバーフロー仕様
- `/dev/serial/by-id/...` で参照可能な安定デバイス名
- 測距更新周期
- `-1` 以外の異常値が出る可能性
