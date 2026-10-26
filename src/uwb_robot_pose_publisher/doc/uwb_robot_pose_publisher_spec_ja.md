# UWB Robot Pose Publisher 仕様書

Version: 0.1 draft  
Date: 2026-09-16  
Status: 初期実装済み。時刻閾値・校正・実機精度は評価待ち。

## 1. 目的と合意事項

UWBタグのWorld位置とロボットのyawから、ロボット中心のWorld位置・姿勢を算出する。
車体を上から見た幾何学的中心と旋回中心は一致しており、この点をロボット位置の基準とする。

合意済みの取り付け位置は、中心から前方に33.5 cm、右に1 cmである。
設定は機体に固定された値とし、ロボットが旋回しても変更しない。

本書のインターフェース、同期方式に沿って初期実装を行った。第9節の項目は実機接続・評価時に確認する。

## 2. 責務と配置

独立したROS 2 Pythonパッケージ `uwb_robot_pose_publisher` として配置し、ノード名も同名とする。

```text
src/uwb_robot_pose_publisher/
├── doc/uwb_robot_pose_publisher_spec_ja.md
├── uwb_robot_pose_publisher/    # 中心補正の計算とROSノード
├── config/                    # パラメータYAML
├── launch/                    # launchファイル
└── test/                      # 検証
```

`uwb_position_publisher` は引き続き距離からタグ位置を算出する。本ノードは中心への幾何学的補正を担当し、Visionとの切り替え・融合、速度指令生成、追加の位置平滑化は担当しない。

```text
/uwb/position ──┐
               ├─ uwb_robot_pose_publisher ─ /uwb/robot_pose
/zed/heading ───┘
```

将来はUWB由来の姿勢とVision由来の姿勢を後段の自己位置推定ノードへ渡し、その出力を `/robot/pose` とする構成を想定する。

## 3. 座標系と補正式

### 3.1 定義

- WorldのX・YはUWBアンカー座標と同じ平面座標系とする。Zは上向きとする。
- ロボット座標は中心を原点とし、xは前方、yは左方、zは上方とする。
- 取り付け位置を `f = tag_offset_forward_m`、`l = tag_offset_left_m` とする。
- 入力yawを `θ = robot_yaw_rad` とする。既存制御の前方軸の式に合わせ、θ=0で前方はWorldの+Y、θが増えると+Yから−X方向に旋回する。
- 出力のWorld yawを `ψ = wrap_pi(θ + π/2)` とする。Worldの+Xをゼロ、+Z軸まわりの反時計回りを正とする。

この対応には、headingのゼロ方向がアンカーWorldの+Yに校正されていることが必要である。磁気方位を受信するだけではWorldへの整合は保証されない。

### 3.2 計算

```text
offset_world_x = cos(ψ) * f - sin(ψ) * l
offset_world_y = sin(ψ) * f + cos(ψ) * l

center_x = tag_x - offset_world_x
center_y = tag_y - offset_world_y
```

取り付け設定は `f=0.335`、`l=-0.010` [m] とする。
θ=0ではタグは中心からWorldの(+0.010, +0.335) [m]にあり、中心はタグから(−0.010, −0.335) [m]となる。

平面移動を対象とし、roll・pitchによるオフセット回転は扱わない。タグとアンカーの高低差による距離補正は既存の `uwb_position_publisher` の責務とし、引き続きタグの高さを用いる。

既存制御の `error_body_y` の式は、θ=0でWorldの+Xを正としている。本書の「左が正」とは異なるため、制御接続時には横方向の符号を確認する。本ノードでは既存の横方向指令の符号を流用しない。

## 4. 入出力

| 方向 | Topic（既定案） | 型 | 使用内容 |
| --- | --- | --- | --- |
| Subscribe | `/uwb/position` | `uwb_interfaces/msg/UwbPosition` | `header.stamp`, `x_m`, `y_m`, `valid` |
| Subscribe | `/zed/heading` | `zed_interfaces/msg/ZedHeading` | `header.stamp`, `robot_yaw_rad`, `valid` |
| Publish | `/uwb/robot_pose` | `geometry_msgs/msg/PoseStamped` | 中心のWorld位置・姿勢 |

出力の規則は以下とする。

- `header.stamp` は対応するUWB入力のstampとし、publish時刻への置き換えは行わない。
- `header.frame_id` は `world_frame_id` とする。
- `position.x/y` に中心座標、`position.z` に平面モデル上の `0.0` を設定する。zは高さの測定値ではない。
- Quaternionは `(x,y,z,w)=(0,0,sin(ψ/2),cos(ψ/2))` とする。
- このメッセージには `child_frame_id` がないため、姿勢の対象は本書で定義するロボット中心座標系とする。
- 初期実装ではTFをpublishしない。

既存のUWB位置は距離メッセージのheaderを継承し、frame名は既定で `uwb` となる。headingのframe名は既定で `zed2i_mag` である。これらの文字列はWorldとの整合を保証しない。本ノードでframe名を設定する前提として、アンカー座標とheadingの基準方向が同一Worldに整合していることを確認する。

## 5. 時刻対応とpublish条件（設計案）

UWB入力ごとに最大1件の姿勢を生成する。headingだけが更新されても、古いUWB位置を再利用した新規姿勢はpublishしない。

1. 有効かつ有限値のheadingをstamp順の短時間バッファに保持する。
2. UWB入力時、そのstampを挟む2件のheadingがあれば、最短角度差を用いてyawを線形補間する。完全に同じstampのheadingがあればその値を使う。
3. 補間に用いる両端のstampは、それぞれUWBのstampとの差が `max_heading_time_diff_sec` 以下でなければならない。
4. 対応するheadingがまだなければ、UWBを最大 `heading_wait_timeout_sec` 保留する。期限までに対応が得られなければ破棄し、外挿や無条件の最新yawへの置き換えはしない。
5. 入力の古さはROS時刻とstampの差で判定する。保留中も鮮度条件を適用する。無効値、非有限値、期限切れの場合は姿勢をpublishしない。
6. 同一入力系列の重複・逆順stampは破棄する。ROS時刻が後退した場合はバッファと保留入力を破棄して再初期化する。未来stampは初期案では不正入力として破棄する。

入力が `valid=false` になった場合、その系列の履歴と保留UWBをクリアし、無効区間をまたぐ補間を防ぐ。復帰後は新しい有効データで再開する。

現在のheadingノードはメッセージ生成時のROS時刻を付けており、センサ観測時刻をそのまま使っていない。UWB位置も受信側で付けたstampを継承し、さらに移動平均後の位置に最新入力のstampを付けている。そのため、上記はheader時刻の対応であり、真の観測時刻同期を保証しない。

特に旋回中は、平滑化されたタグ位置とyawの遅延差によって中心が揺れる可能性がある。初期の幾何学検証では上流の `moving_average_window=1` を使用し、実運用の平滑化・時刻改善は別途評価する。`device_time_ms` はROS時刻への対応が未定義のため、本ノードの同期に直接使用しない。

## 6. パラメータ案

設定は起動時にYAMLから読み込み、実行中の動的変更は初期実装の対象外とする。

| Parameter | 設定値／既定案 | 条件・意味 |
| --- | --- | --- |
| `uwb_position_topic` | `/uwb/position` | 空文字不可 |
| `zed_heading_topic` | `/zed/heading` | 空文字不可 |
| `robot_pose_topic` | `/uwb/robot_pose` | 空文字不可 |
| `world_frame_id` | `world` | 空文字不可、アンカー座標系と一致させる |
| `tag_offset_forward_m` | `0.335` | 有限値、前が正 |
| `tag_offset_left_m` | `-0.010` | 有限値、左が正 |
| `position_timeout_sec` | `0.5`（暫定） | 正の有限値 |
| `heading_timeout_sec` | `0.5`（暫定） | 正の有限値 |
| `max_heading_time_diff_sec` | `0.1`（暫定） | 正の有限値、補間両端の許容時刻差 |
| `heading_wait_timeout_sec` | `0.1`（暫定） | 正の有限値、heading到着の待機上限 |
| `heading_buffer_duration_sec` | `1.0`（暫定） | 正の有限値、時刻対応に必要な履歴を保持する |

取り付け値以外の数値は実機測定で見直す。バッファ期間は少なくとも `position_timeout_sec + max_heading_time_diff_sec` を満たすように検証する。heading履歴と保留UWBはそれぞれ最大1024件とし、超過時は最古の要素を破棄して警告する。期限切れデータはタイマーで継続的に除去する。新しい姿勢を出力できた場合、それより古い未対応UWBは破棄し、出力stampの逆行を防ぐ。

## 7. 異常時と下流への契約

不正なパラメータは起動エラーとする。入力不足、invalid、非有限値、時刻不整合、期限切れでは出力を停止し、理由を頻度制限付きログへ出す。過去の正常姿勢を新しいstampで再送しない。

`PoseStamped` にはvalidフラグがないため、下流は出力のstampによるtimeoutを必須とする。本ノードの無出力だけで即時の異常通知はできない。即時通知が必要なら別途状態topicを検討する。

## 8. 検証と既存システムへの接続

| 検証 | 期待結果 |
| --- | --- |
| オフセットゼロ | 中心位置とタグ位置が一致する |
| θ=0、タグ位置(1, 2) m | 中心位置(0.990, 1.665) m、出力yaw=π/2 |
| θ=π/2、タグ位置(1, 2) m | 中心位置(1.335, 1.990) m、出力yawは±π相当 |
| 中心固定でタグを理想的に一周させる | 全yawで補正後中心が一定となる |
| yawが+πと−πをまたぐ補間 | ゼロ方向へ大回りせず最短角度で補間する |
| 無効入力・古い入力・同期不可 | 正常姿勢をpublishしない |
| headingのみ継続、UWB停止 | 古い位置から新規姿勢を生成しない |
| 実機のその場旋回 | タグ軌跡と中心軌跡を比較し、残差をUWB・yaw・時刻の誤差とともに評価する |

実機の中心揺れの許容値は、現時点では未確定とする。

`uwb_position_zed_pose_ctrl` は `/uwb/robot_pose` を購読し、中心位置を用いて制御する。入力QuaternionのWorld yawからπ/2を引き、既存の制御・目標yaw基準（0でWorldの+Y）に合わせる。frame・Quaternion・stampを検証し、姿勢が古い場合は停止する。

`uwb_zed_docking.launch.py` と `managed_docking.launch.py` は本ノードを起動する。全体の `docking.launch.py` はmanaged launch経由で1つ起動する。

## 9. 実機接続・評価時に確認する事項

1. Worldの+Yとheadingのゼロ方向を合わせる校正手順、横方向の実機符号。
2. header時刻の補間方式と暫定閾値の妥当性。観測時刻の改善を初期実装に含めるか。
3. 上流の移動平均による遅延の扱いと、実機評価時のフィルタ設定。
4. 中心揺れの合格基準。
5. 接続済み制御での実機動作と、必要なら状態topicの追加。

## 10. 参照した現行実装

- [UWB位置算出](../../uwb_position_publisher/uwb_position_publisher/uwb_position_publisher.py)
- [UWB距離入力](../../uwb_position_publisher/uwb_position_publisher/uwb_distance_publisher.py)
- [ZED heading算出](../../zed_heading_publisher/zed_heading_publisher/zed_heading_publisher.py)
- [既存位置制御の座標変換](../../uwb_position_zed_pose_ctrl/uwb_position_zed_pose_ctrl/pose_control.py)

## 11. ビルドと起動

ワークスペース `ptp_navi` で実行する。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to uwb_robot_pose_publisher
source install/setup.bash
ros2 launch uwb_robot_pose_publisher uwb_robot_pose_publisher.launch.py
```

このlaunchは中心補正ノードのみを起動する。UWB位置・ZED headingの入力ノードは別途起動する。独自YAMLを使う場合は `config_file:=/absolute/path/config.yaml` を指定する。

```bash
ros2 topic echo /uwb/robot_pose
colcon test --packages-select uwb_robot_pose_publisher
colcon test-result --verbose
```

起動時パラメータはread-onlyとして宣言し、実行中の変更を拒否する。出力は `PoseStamped` で、共分散は含まない。
