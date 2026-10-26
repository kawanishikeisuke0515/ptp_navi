# Path Control 全体仕様

2026-10-26｜実装仕様・実機未検証

添付の「直線 Path Control 仕様書」ドラフトv1.0を基に、採用済みの制御周期・回転速度上限・初期化タイムアウトと、deadmanを使用しない構成を反映した。ファイル名は既存リンクを維持するため従来のままとする。未決定事項は末尾にまとめる。

## 1. 目的とシステム構成

全方向移動ロボットを、始点Aから終点Bへ向かう直線経路に沿って走行させる。経路方向の移動、横ずれ補正、車体方向の補正を同時に行う。目標車体方向は走行中、A→Bの方向に固定する。

```text
UWB機器（有効5 Hz）
  ↓ シリアル距離
uwb_position_publisher（距離取得＋位置推定＋5点移動平均）
  ↓ /uwb/position
  ├──────────────────────────────┐
ZED → zed_heading_publisher → /zed/heading   │
  └──────────────────────────────┤
                    uwb_robot_pose_publisher
                    時刻同期・タグ位置から車体中心への補正
                              ↓ /uwb/robot_pose
                    path_control（本仕様・20 Hz）
                              ↓ /rov_cmd_vel
                    locomotion_core_rpm
                    Twist → 車輪RPM（20 Hz）
                              ↓ /rov/motor_rpm
                    Roboteqドライバ → モーター
```

上図の位置・方位はともに `uwb_robot_pose_publisher` に入力する。Path Controlへの統合入力を `/uwb/robot_pose` とする想定。Jetson電力配信は監視用途であり、経路制御の必須入力にはしない。

## 2. 座標系と入出力

ワールド座標の+x方向を姿勢角0、反時計回りを正とする。車体座標は前方+x、左方向+y、反時計回りの回転+z。位置と終点は同じUWBワールド座標系で表し、単位はm、姿勢角はrad。

| 方向 | トピック・設定 | 型・内容 |
| --- | --- | --- |
| 入力 | `/uwb/robot_pose` | `geometry_msgs/PoseStamped`。補正済み車体中心位置PとQuaternion姿勢。実効約5 Hz |
| 設定 | 終点B = (goal_x, goal_y) | 同じワールド座標系の終点[m]。YAMLファイルのROSパラメーター `goal_x`・`goal_y` で起動時に指定する |
| 算出 | 始点A | 起動・復帰後の新規有効車体中心位置10個の平均。走行中固定 |
| 出力 | `/rov_cmd_vel` | `geometry_msgs/Twist`。linear.x/y [m/s]、angular.z [rad/s]。20 Hz |
| 出力 | 状態・異常通知 | 状態、異常理由、解除を通知。`/path_control/status`（std_msgs/String、JSON）、20 Hz |

`/deadman` は使用しない。Path Controlが非ゼロ指令を出すと下位系が駆動する。INIT・ERROR・GOAL_CHECK・COMPLETEではゼロ指令を継続する。指令の発行元は一つにする。別の開始操作は設けず、起動後の初期化が完了したら自動で走行を開始する。ERRORからの復帰時もINITでの再初期化完了後に自動で再開する。

### 座標整合の確認点

既存の `uwb_robot_pose_publisher` は、ZEDの `robot_yaw_rad` にπ/2を加えた角度でタグ補正と姿勢出力を行う。これがUWBワールド+x方向＝yaw 0と一致するか実機で確認する。Path Control側で同じ補正を重ねない。

校正ツールは校正開始時の向きを基準としたzero_heading_degを生成する。その開始方向とUWBワールド軸の対応も確認が必要。

### 終点Bの設定

終点は起動時にYAMLから読み込む `goal_x`・`goal_y` [m] で指定する。以下は設定書式の例で、座標は実際の目標に置き換える。ノード名は `path_control` を想定する。

```yaml
path_control:
  ros__parameters:
    goal_x: 1.0
    goal_y: 2.0
```

終点Bは走行中・ERRORからの復帰時とも維持する。パラメーターは起動時固定。COMPLETE後の再走行は設定変更後にノードを再起動する。

## 3. 採用パラメーター

| 項目 | パラメーター名案 | 初期値 |
| --- | --- | --- |
| 制御周期 | `control_rate_hz` | 20 Hz（50 ms周期） |
| 通常の経路方向速度 | `path_speed_m_s` | 0.3 m/s。通過後は−0.3 m/s |
| 並進速度の下限 | `min_linear_speed_m_s` | 0.05 m/s。移動中のみ |
| 並進速度の上限 | `max_linear_speed_m_s` | 0.5 m/s |
| 横ずれ補正ゲイン | `cross_track_gain` | 1.0 [1/s] |
| 角度補正ゲイン | `heading_gain` | 1.0 [1/s] |
| 回転速度上限 | `max_angular_speed_rad_s` | 0.3 rad/s（±0.3に制限） |
| 初期化タイムアウト | `init_timeout_sec` | 10.0秒 |
| 位置の更新停止判定 | `position_timeout_sec` | 0.6秒 |
| 姿勢の更新停止判定 | `heading_timeout_sec` | 0.6秒 |
| 始点の平均サンプル数 | `init_sample_count` | 10個 |
| 到着許容距離 | `goal_tolerance_m` | 0.10 m |
| 到着確認時間 | `goal_hold_sec` | 2.0秒 |

上記の名称で実装。終点近傍での減速、加速度制限、Path Control独自の車輪RPM制限は設けない。下位の `locomotion_core_rpm` による車輪RPM上限制限は引き続き適用される。

## 4. 始点Aの初期化

INITに入った時点から10秒を計測する。車体は停止指令を維持し、新規の有効な車体中心位置をN個取得する。

```text
A.x = Σ x_i / N
A.y = Σ y_i / N
N = 10（変更可能）
```

UWBの有効更新は5 Hz。別々の新規サンプルだけを数え、20 Hzの制御周期で同じ位置を重複計上しない。10個の取得は初回から最短約1.8秒。無効サンプルは平均に含めない。

10秒以内に揃わなければERRORへ移行する。取得後のAは走行中固定する。Bから0.10 m以内なら経路単位ベクトルを計算せずGOAL_CHECKへ進む。それ以外はA→Bの経路を作成してDRIVEへ進む。ERRORから復帰した場合はAと収集中サンプルを破棄し、再取得する。Bは維持する。

## 5. 経路追従の制御式

### 5.1 経路方向への移動

```text
L = sqrt((B.x − A.x)^2 + (B.y − A.y)^2)
t = (B − A) / L = (t_x, t_y)
n = (−t_y, t_x)
d_remaining = (B − P) · t

許容範囲外かつ d_remaining ≥ 0：v_forward = +0.3 m/s
許容範囲外かつ d_remaining < 0：v_forward = −0.3 m/s
v_path = v_forward · t
```

tは経路の進行方向、nは経路の左向き単位ベクトル。終点通過後は同じ経路に沿って戻り、車体の目標方向はA→Bのままとする。

### 5.2 横ずれ補正

```text
e_CT = (P − A) · n
v_CT = −K_CT · e_CT · n
v_world = v_path + v_CT
K_CT = 1.0 [1/s]
```

経路左側のずれを正とする。正なら右向き、負なら左向きに補正する。

### 5.3 並進速度制限と車体座標変換

```text
s = norm(v_world)
移動中かつ s > 0：
    v_world = v_world · clamp(s, 0.05, 0.5) / s
s = 0 の場合：v_world = (0, 0)

linear.x =  cos(theta) · v_world.x + sin(theta) · v_world.y
linear.y = −sin(theta) · v_world.x + cos(theta) · v_world.y
```

合成速度の方向を維持して大きさを制限する。停止状態では最低速度を適用しない。回転変換で速度の大きさは変わらない。

### 5.4 車体方向の補正

```text
theta_AB = atan2(B.y − A.y, B.x − A.x)
e_theta = atan2(sin(theta_AB − theta), cos(theta_AB − theta))
angular.z = clamp(K_theta · e_theta, −0.3, +0.3)
K_theta = 1.0 [1/s]
```

短い方向に回転する。横ずれ補正中も車体方向はA→Bのままとし、移動方向だけが斜めになる。

## 6. 到着判定

```text
d_goal = norm(B − P)
d_goal ≤ 0.10 m → GOAL_CHECK（並進・回転ともゼロ）
```

新規有効位置で範囲内が2秒以上継続したことを確認してCOMPLETEへ進む。同じ位置を保持したまま壁時計で2秒経過しても完了にしない。新規位置の時刻で継続時間を評価し、入力の鮮度も満たす必要がある。

範囲外なら確認時間をリセットしDRIVEへ戻る。異常ならリセットしERRORへ移行する。COMPLETE後は停止を維持する。停止は速度指令ゼロを意味し、実際の車輪停止を確認する機能は現構成にない。

## 7. 入力異常と復帰

位置または姿勢が0.6秒以上更新されなければERRORへ移行し、ゼロ指令と異常通知を維持する。INITの未受信待機は10秒の初期化制限を適用する。

異常発生後の新しい有効データで位置・姿勢の両方が正常であることを確認したら解除を通知し、必ずINITへ戻る。過去のA、平均サンプル、到着確認時間を破棄し、Bは維持する。

統合Poseにはvalidフラグがなく、上流は無効入力や同期失敗時にPoseを配信しない。統合Poseのみを購読する場合、位置・姿勢の更新停止は共通のPose更新停止として検出する。原因を区別するには元の `/uwb/position` と `/zed/heading` の監視または上流の状態通知が必要。

非有限値、不正Quaternion（ノルム誤差0.01超）、frame_id不一致、重複・逆順・未来stampをERRORとする。受信停止とheader.stampの古さをともに判定し、ROS時刻の逆行時も停止して再初期化する。異常入力を初期化・到着確認の有効サンプルに含めない。

## 8. 状態遷移

| 状態 | 出力・処理 | 遷移 |
| --- | --- | --- |
| INIT | ゼロ指令、新規位置10個を平均 | 取得完了→DRIVEまたはGOAL_CHECK。10秒超過・入力異常→ERROR |
| DRIVE | 経路速度・横ずれ・角度補正、速度制限 | 到着範囲内→GOAL_CHECK。入力異常→ERROR |
| GOAL_CHECK | ゼロ指令、新規位置で2秒継続を確認 | 継続成立→COMPLETE。範囲外→DRIVE。入力異常→ERROR |
| ERROR | ゼロ指令、異常通知、確認時間を破棄 | 新規の正常位置・姿勢を確認→INIT |
| COMPLETE | ゼロ指令を維持 | 再走行にはノード再起動 |

処理順は異常判定・停止条件を速度生成より優先する。INIT・DRIVE・GOAL_CHECKの入力異常時は到着判定よりERRORを優先する。COMPLETEは入力状態にかかわらずゼロ指令を維持する。

## 9. 更新周期・遅延と下位系

制御演算とTwist配信は20 Hz、位置情報は実効5 Hz。新規入力の間は最後の有効値を保持するが、更新停止判定を継続する。更新レートを上げても位置計測そのものは増えない。

既存UWB位置推定は5点移動平均を使用する。5 Hzでは定常的な遅れが約0.4秒あり、0.3 m/s走行時に約0.12 mに相当する。平均位置には最新入力の時刻が付くため、この遅れは単純なstamp鮮度判定で検出できない。現在の方位との組み合わせによる時間的なずれも実機で評価する。

`uwb_robot_pose_publisher` の既存入力タイムアウト0.5秒、方位時刻差上限0.1秒、方位待機上限0.1秒は変更していない。Path Controlの0.6秒はその出力を監視する別の制限であり、センサー異常から車輪停止までの総時間を意味しない。

下位系ではTwist入力が0.5秒途絶えるとゼロRPMを配信する。RPM入力が0.5秒途絶えるとドライバが停止し、新しい有効なRPMで再開する。通信異常はラッチし、原因解消後のドライバ再起動を要する。

## 10. 未決定事項

- UWB軸とZED方位の整合、校正時の基準方向、π/2補正の実機確認。
- 5点移動平均の遅延に対する扱い。到着判定のヒステリシス等は実機評価後に検討する。
- 実停止の確認方法、LiDARによる停止・回避との優先順位。

## 11. 検証計画

1. 座標変換、横ずれ符号、角度折り返し、並進・回転の上限制限を単体検証する。
2. 新規位置のみを数える初期化、10秒タイムアウト、ERROR復帰時のA再取得を検証する。
3. 終点手前・通過後の速度符号、到着中の離脱、欠測時に完了しないことを検証する。
4. 入力停止・無効値でゼロ出力になり、古いサンプルで復帰しないことを検証する。
5. 実機で方位基準と左右・回転符号を確認し、低速で各制御成分を個別評価する。
6. 合成走行で位置、姿勢、e_CT、e_theta、速度指令、状態、入力時刻を記録し、停止距離と到着確認を評価する。

元資料はユーザー提供の `path_control_spec.html`（ドラフトv1.0）。本書は設計の整理であり、ノードや実機動作を実装・検証したことを示すものではない。

## 12. 実装時の確定事項

- 統合Poseのみ購読し、元センサーの異常原因の分類は行わない。新しい正常Poseの受信でERRORを解除し、そのサンプルからINITの平均を再取得する。
- 状態通知はJSON文字列。state、reason、error、init_samples、start、goalを20 Hzで配信する。reasonはpose_timeout、init_timeout、nonfinite_pose、pose_timestamp、duplicate_or_reversed_stamp、frame_mismatch、invalid_quaternion、clock_backwards、degenerate_path。解除時は空文字列。
- 非ゼロの速度指令は20 Hz。停止状態への入力遷移時はゼロ指令を即時追加配信する。
- 終点パラメーターの初期値は未設定扱いとし、YAMLにgoal_x・goal_y両方が必要。全パラメーターをread-onlyとする。
- 経路長が1e-9 m未満なのに移動が必要になった場合はdegenerate_pathで停止する。
- ノード正常終了時はゼロ指令を送信する。終了時の配信到達は保証できないため、下位のタイムアウトも継続する。
