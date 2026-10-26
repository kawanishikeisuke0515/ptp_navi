# ZED Wrapper Data Hub 設計仕様書

Version: 0.2 draft
Date: 2026-09-15
Status: draft（リポジトリ内のlaunch・設定に基づく。実機の全出力は未照合）

## 1. 概要・適用範囲

`zed_wrapper_data_hub` は、公式 `zed_wrapper` を専用設定で起動し、ZEDカメラの画像・センサー・自己位置推定・状態情報をROS 2トピックとして提供する独立したROS 2パッケージである。本仕様書は、本パッケージの構成、依存関係、起動引数、配信設定、トピックのフィールド、単体確認方法を定義する。

デフォルトの対象カメラはZED2iである。取得・推定・配信処理は公式wrapperが担当し、本パッケージには独自のデータ処理ノードやメッセージ型はない。

## 2. パッケージ構成・依存関係

```text
zed_wrapper_data_hub/
├── package.xml
├── setup.py / setup.cfg
├── resource/zed_wrapper_data_hub
├── zed_wrapper_data_hub/__init__.py
├── launch/zed_data_hub.launch.py
├── config/zed2i_data_hub.yaml
└── docs/
    ├── zed_wrapper_data_hub_design_ja.md
    ├── zed_wrapper_data_hub_design_ja.html
    ├── zed_wrapper_data_hub_design_en.md
    └── zed_wrapper_data_hub_design_en.html
```

ビルド形式は `ament_python`。実行依存は `launch`、`launch_ros`、`zed_wrapper`、`zed_description` である。公式wrapperが必要とするZED SDK、ドライバー、対応カメラも実行環境側に用意する。launch・config・docsはパッケージのshareディレクトリにインストールされる。

### 2.1 環境構築・ソース取得

ROS 2、ZED SDK、対応するCUDA・ドライバーを事前にインストールする。wrapperの対応SDK・ROSディストリビューションはバージョンによって異なるため、[公式導入手順](https://github.com/stereolabs/zed-ros2-wrapper#installation)と[リリース](https://github.com/stereolabs/zed-ros2-wrapper/releases)を確認する。本仕様書では実機で検証済みの組み合わせはまだ固定していない。

以下は `ptp_navi` ワークスペースに両リポジトリをソース配置する手順である。`/path/to/ptp_navi` は実際の場所に、`humble` は使用するROSディストリビューションに置き換える。すでに取得済みのリポジトリについてはcloneを省略する。

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ptp_navi

git clone https://github.com/stereolabs/zed-ros2-wrapper.git src/zed-ros2-wrapper
git clone https://github.com/stereolabs/zed-ros2-interfaces.git src/zed-ros2-interfaces
```

配置は次のようになる。

```text
ptp_navi/src/
├── zed_wrapper_data_hub/
├── zed-ros2-wrapper/
└── zed-ros2-interfaces/
```

`zed-ros2-interfaces` は `zed_msgs` を提供する。公式手順では環境によってバイナリパッケージによる導入も可能だが、ここでは両方をソースからビルドする。異なるディレクトリに同じ `zed_msgs` パッケージを重複配置しない。

clone直後は取得時点のデフォルトブランチになる。ビルド前にSDKに対応するwrapperのタグ・コミットと、それに対応するinterfacesの版を選択する。以下のプレースホルダーは実際の値に置き換える。

```bash
git -C src/zed-ros2-wrapper checkout <wrapper-tag-or-commit>
git -C src/zed-ros2-interfaces checkout <interfaces-tag-or-commit>
```

### 2.2 依存関係の導入・ビルド

`rosdep` と `colcon` が未導入の場合は、先に導入する。

```bash
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions
# rosdep未初期化の環境でのみ実行
sudo rosdep init
```

ROS 2環境を読み込んだターミナルで、ワークスペース直下から実行する。

```bash
cd /path/to/ptp_navi
rosdep update
rosdep install --from-paths src/zed_wrapper_data_hub src/zed-ros2-wrapper src/zed-ros2-interfaces --ignore-src -r -y
colcon build --symlink-install --packages-up-to zed_wrapper_data_hub --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--packages-up-to` はHubとワークスペース内の依存パッケージをビルドする。ZED SDKやCUDAは上記のcloneだけでは導入されない。依存解決やビルドに失敗した場合はエラーを解消してから起動する。

```bash
ros2 pkg prefix zed_wrapper
ros2 pkg prefix zed_description
ros2 pkg prefix zed_msgs
ros2 pkg prefix zed_wrapper_data_hub
ros2 launch zed_wrapper_data_hub zed_data_hub.launch.py
```

別ターミナルでもROS 2と `install/setup.bash` を読み込む。再現用に、動作確認したROSディストリビューション・SDKバージョンと、次のコミットIDを記録する。

```bash
git -C src/zed-ros2-wrapper rev-parse HEAD
git -C src/zed-ros2-interfaces rev-parse HEAD
```

## 3. 起動構成と責務

```text
zed_data_hub.launch.py
  └── zed_wrapper / zed_camera.launch.py
        ├── ZEDカメラのopen・grab
        ├── 画像・センサー・自己位置推定・状態の配信
        └── カメラモデルとTFの提供（起動引数に従う）
```

本パッケージは `ros_params_override_path` に設定YAMLを渡し、公式launchをincludeする。デフォルトでは `config/zed2i_data_hub.yaml` を使用する。トピックの生成、メッセージへの値の設定、実際の配信条件は使用中のwrapperに依存する。

## 4. 起動方法・起動引数

パッケージをビルドし、ROS 2とワークスペースの環境を読み込んだ状態で起動する。

```bash
ros2 launch zed_wrapper_data_hub zed_data_hub.launch.py
```

別設定を使用する場合：

```bash
ros2 launch zed_wrapper_data_hub zed_data_hub.launch.py ros_params_override_path:=/absolute/path/to/custom.yaml
```

| Argument | Type | Default | 内容 |
| --- | --- | --- | --- |
| `camera_model` | string | `zed2i` | カメラモデル |
| `camera_name` | string | `zed2i` | カメラ名・標準namespace |
| `namespace` | string | 空文字 | wrapperに渡すnamespace |
| `node_name` | string | `zed_node` | wrapperノード名 |
| `serial_number` | string | `0` | シリアル番号。0はwrapper標準動作 |
| `camera_id` | string | `-1` | デバイスID。-1はwrapper標準動作 |
| `publish_urdf` | bool | `true` | カメラURDFとrobot_state_publisherの起動 |
| `publish_tf` | bool | `true` | odomからカメラへのTF |
| `publish_map_tf` | bool | `true` | mapからodomへのTF |
| `publish_imu_tf` | bool | `false` | wrapperのIMU TF配信 |
| `enable_ipc` | bool | `true` | プロセス内通信 |
| `use_sim_time` | bool | `false` | シミュレーション時刻の使用 |
| `sim_mode` | bool | `false` | シミュレーションモード |
| `ros_params_override_path` | string | パッケージ内 `config/zed2i_data_hub.yaml` | 上書き設定YAML |

## 5. 現在のデフォルト設定

本節は `config/zed2i_data_hub.yaml` とlaunchのデフォルト値に対応する。別YAMLや起動引数で上書きした場合は異なる。配信ONは設定上の有効化を意味し、実機での正常受信や指定周期を保証するものではない。

### 5.1 有効な情報・機能

| 情報・機能 | 設定 |
| --- | --- |
| RGB画像 | `video.publish_rgb: true` |
| IMU | `sensors.publish_imu: true` |
| 磁場 | `sensors.publish_mag: true` |
| 位置推定・odom / pose | `pos_tracking_enabled: true`、`publish_odom_pose: true` |
| 状態情報 | `general.publish_status: true` |
| TF | `publish_tf: true`、`publish_map_tf: true` |
| カメラモデル | launchの `publish_urdf: true` |

### 5.2 無効な配信・機能

| 情報・機能 | 設定（すべてfalse） |
| --- | --- |
| 左右画像・RAW・グレースケール・ステレオ画像 | `video.publish_left_right`、`publish_raw`、`publish_gray`、`publish_stereo` |
| RAW IMU・カメラ/IMU変換情報・気圧・温度 | `sensors.publish_imu_raw`、`publish_cam_imu_transf`、`publish_baro`、`publish_temp` |
| 深度画像・深度情報・点群・深度信頼度・視差 | `depth.publish_depth_map`、`publish_depth_info`、`publish_point_cloud`、`publish_depth_confidence`、`publish_disparity` |
| エリアメモリ・3Dランドマーク・共分散付きposeの別出力・軌跡 | `pos_tracking.area_memory`、`publish_3d_landmarks`、`publish_pose_cov`、`publish_cam_path` |
| マッピング・検出平面 | `mapping.mapping_enabled`、`publish_det_plane` |
| 物体検出 | `object_detection.od_enabled` |
| ボディ追跡 | `body_tracking.bt_enabled` |
| ストリーミングサーバー | `stream_server.stream_enabled` |
| wrapperのIMU TF | launchの `publish_imu_tf` |

`publish_pose_cov: false` は別の共分散付きpose出力を無効にする設定であり、Odometryのメッセージ構造から `covariance` フィールドを取り除くものではない。

### 5.3 取得条件・推定条件

| 項目 | 現在値 |
| --- | --- |
| カメラ名・モデル | `zed2i` |
| 取得解像度・フレームレート | `HD720`、`grab_frame_rate: 30` |
| 処理FPS上限 | `grab_compute_capping_fps: 30.0` |
| 配信解像度 | `pub_resolution: CUSTOM`、`pub_downscale_factor: 2.0` |
| 画像配信FPS | `pub_frame_rate: 15.0` |
| センサー配信レート | `sensors_pub_rate: 50.0` |
| 深度計算 | `depth_mode: NEURAL_LIGHT` |
| 点群配信レート設定 | `point_cloud_freq: 5.0`（点群配信自体はOFF） |
| 追跡モード・IMU融合 | `pos_tracking_mode: AUTO`、`imu_fusion: true` |
| 推定次元 | `two_d_mode: true`、`fixed_z_value: 0.0` |
| 基準フレーム | `map_frame: map`、`odometry_frame: odom` |

深度出力は無効だが、位置推定を維持するため深度計算は `NEURAL_LIGHT` で有効にしている。設定ファイルのコメントでは `NONE` にすると公式wrapperで位置推定が無効になる場合があるとしている。現在の自己位置推定は2D設定であり、自由な3D運動の出力を目的とする設定ではない。

## 6. トピックとフィールド

### 6.1 トピック確認記録

既存の実機確認記録（2026-09-01）では、`camera_name:=zed2i`、`node_name:=zed_node` で以下のトピックが確認されている。名称はnamespace・wrapperのバージョンに依存する。CameraInfoの2つのパスは記録上の候補であり、使用環境で配信元と型を確認する。

| Topic | データ内容 |
| --- | --- |
| `/zed2i/zed_node/rgb/color/rect/image` | 補正済みカラー画像 |
| `/zed2i/zed_node/rgb/color/rect/camera_info` | カメラ校正情報 |
| `/zed2i/zed_node/rgb/color/rect/image/camera_info` | カメラ校正情報の別パス候補 |
| `/zed2i/zed_node/imu/data` | IMU姿勢・角速度・加速度 |
| `/zed2i/zed_node/imu/mag` | 磁場 |
| `/zed2i/zed_node/odom` | 推定位置・姿勢・速度 |
| `/zed2i/zed_node/pose` | 推定位置・姿勢 |
| `/zed2i/zed_node/pose/status` | 位置推定の状態 |
| `/zed2i/zed_node/status/health` | カメラの健全性 |
| `/zed2i/zed_node/status/heartbeat` | 動作通知 |
| `/zed2i/joint_states` | カメラモデルの関節状態 |
| `/zed2i/zed2i_description` | カメラのURDF記述 |

### 6.2 トピックの型とフィールド

以下では `/zed2i/zed_node` を接頭辞から省略する。標準メッセージの構造と、上流の `zed_msgs` 定義を示す。6.1 の記録はトピック名の実機確認であり、以下の全フィールドを実機検証済みという意味ではない。実際の型は 6.6 のコマンドで照合する。特に wrapper 固有の型・状態コードはインストールされたバージョンの定義を優先する。

| Topic | メッセージ型（想定） | フィールドと内容 |
| --- | --- | --- |
| `/rgb/color/rect/image` | `sensor_msgs/msg/Image` | `header`、`height` / `width`（画素数）、`encoding`（画素形式）、`is_bigendian`、`step`（1行のバイト数）、`data`（画素バイト列） |
| `/rgb/color/rect/camera_info` または `/rgb/color/rect/image/camera_info` | `sensor_msgs/msg/CameraInfo` | `header`、`height` / `width`、`distortion_model`、`d`（歪み係数）、`k`（3×3内部行列）、`r`（3×3補正回転行列）、`p`（3×4投影行列）、`binning_x/y`、`roi`（切り出し領域） |
| `/imu/data` | `sensor_msgs/msg/Imu` | `header`、`orientation`、`angular_velocity`、`linear_acceleration`、各測定の共分散。詳細は 6.4 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | `header`、`magnetic_field.x/y/z`（磁場、T）、`magnetic_field_covariance`（3×3、9要素） |
| `/odom` | `nav_msgs/msg/Odometry` | `header`、`child_frame_id`、`pose.pose`、`pose.covariance`、`twist.twist`、`twist.covariance`。詳細は 6.3 |
| `/pose` | `geometry_msgs/msg/PoseStamped` | `header`、`pose.position.x/y/z`（m）、`pose.orientation.x/y/z/w`（クォータニオン）。速度・共分散は含まない |
| `/pose/status` | `zed_msgs/msg/PosTrackStatus`（要実機照合） | `odometry_status`（VIO状態）、`spatial_memory_status`（マップ内追跡状態）、`status`（参照した上流定義では非推奨） |
| `/status/health` | `zed_msgs/msg/HealthStatusStamped`（要実機照合） | `header`、`serial_number`、`camera_name`、`low_image_quality`、`low_lighting`、`low_depth_reliability`、`low_motion_sensors_reliability`（各異常フラグ）、`scene_illuminance`（0.1 lux単位） |
| `/status/heartbeat` | `zed_msgs/msg/Heartbeat`（要実機照合） | `beat_count`（カウンタ）、`node_ns`、`node_name`、`full_name`、`camera_sn`、`svo_mode`、`simul_mode` |

`header` は `stamp.sec` / `stamp.nanosec`（時刻）と `frame_id`（基準フレーム名）を持つ。画像の光学フレームはX右・Y下・Z前方であり、ロボット本体の軸と混同しない。画像の色順は `encoding` で確認する。画像と CameraInfo の解像度・フレームを照合し、補正済み画像の投影には `p` を参照する。

状態コードはフィールドごとの定数で解釈する。同じ数値でもフィールドによって意味が異なる。ハートビートの受信だけで位置推定が有効とは判断しない。

参照定義：[Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)、[CameraInfo](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg)、[MagneticField](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/MagneticField.msg)、[PosTrackStatus](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/PosTrackStatus.msg)、[HealthStatusStamped](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/HealthStatusStamped.msg)、[Heartbeat](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Heartbeat.msg)。リンク先は上流ブランチであり、実機のバージョンを固定するものではない。

### 6.3 Odometry の構造と速度の読み方

```text
nav_msgs/msg/Odometry
├── header
│   ├── stamp.sec / stamp.nanosec       # データの時刻
│   └── frame_id                       # 位置・姿勢の基準フレーム
├── child_frame_id                     # 推定対象フレーム。速度の表現座標系
├── pose                               # PoseWithCovariance
│   ├── pose                           # Pose：位置・姿勢そのもの
│   │   ├── position.x/y/z             # 位置 [m]
│   │   └── orientation.x/y/z/w        # クォータニオン
│   └── covariance                     # 6×6、36要素
└── twist                              # TwistWithCovariance
    ├── twist                          # Twist：速度そのもの
    │   ├── linear.x/y/z               # 並進速度 [m/s]
    │   └── angular.x/y/z              # 角速度 [rad/s]
    └── covariance                     # 6×6、36要素
```

`twist.twist` の外側は「速度と共分散のセット」、内側は「速度そのもの」である。同じ計算を2回行う意味ではなく、同名の入れ子フィールドを2段たどる。`pose.pose` も同じ構造である。

```python
# msg は受信した Odometry
vx = msg.twist.twist.linear.x       # X方向の並進速度 [m/s]
wz = msg.twist.twist.angular.z      # Z軸まわりの角速度 [rad/s]
x = msg.pose.pose.position.x       # 基準フレーム内のX位置 [m]
velocity_cov = msg.twist.covariance
```

位置・姿勢は `header.frame_id`、速度は `child_frame_id` の座標系で表現される。現在の設定では odometry frame は `odom` だが、実際のフレーム名は受信値で確認する。`linear.x` をロボットの前進速度として使うには、対象フレームのX軸とロボット前方の関係を確認する。クォータニオンの `z` はyaw角そのものではなく、4成分から変換する。

共分散は行優先の6×6行列で、pose は位置x/y/z・X/Y/Z軸まわりの回転、twist は並進速度x/y/z・角速度x/y/zの順である。対角要素の添字は `0, 7, 14, 21, 28, 35`。値が実際に設定されているかは配信側に依存し、ゼロ配列だけで「誤差がない」と判断しない。

Odometry に加速度は含まれない。停止中も推定速度がゼロ付近で揺れる場合があるため、停止判定は実測に基づく閾値と継続時間で行う。更新停止で最後の値が残る場合と、更新中の小さな変動は区別する。速度の精度や算出方法はフィールドの存在だけでは保証されず、実際の移動距離・時間との照合が必要である。

座標系と入れ子構造の定義：[Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg)。

### 6.4 IMU・磁場・Pose の読み方

| IMUフィールド | 意味・単位 |
| --- | --- |
| `header.stamp` / `header.frame_id` | 測定時刻とセンサーフレーム |
| `orientation.x/y/z/w` | 姿勢クォータニオン（無次元） |
| `angular_velocity.x/y/z` | 各軸まわりの角速度（rad/s） |
| `linear_acceleration.x/y/z` | 各軸方向の加速度（m/s²） |
| `orientation_covariance` | 姿勢の共分散（3×3、9要素） |
| `angular_velocity_covariance` | 角速度の共分散（3×3、9要素） |
| `linear_acceleration_covariance` | 加速度の共分散（3×3、9要素） |

IMU の各共分散は行優先で、全要素ゼロは共分散不明、先頭要素 `-1` は対応する測定値が提供されないことを表す。加速度をロボットの移動加速度として利用する前に、センサー軸・取り付け姿勢・wrapper出力の重力成分の扱いを確認する。定義：[Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)。

磁場 `magnetic_field.x/y/z` は `header.frame_id` の軸で表現され、単位はTeslaである。µTへの変換は `値 × 1e6`。`magnetic_field_covariance` の全要素ゼロは共分散不明を表す。このトピックに方位角そのものは含まれない。

`/pose` は `pose.position` と `pose.orientation` を直接持ち、Odometry のような `pose.pose` ではない。現在の設定では map frame は `map`、位置推定は `two_d_mode: true`、`fixed_z_value: 0.0` である。実際の基準は `header.frame_id` で確認する。

### 6.5 TF・モデル関連トピック

以下は完全なトピック名で示す。URDF関連は launch が起動する robot_state_publisher などを含む構成全体の出力である。

| Topic | メッセージ型（想定） | 主要フィールド |
| --- | --- | --- |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | `transforms[]` の各要素に `header`（親フレームと時刻）、`child_frame_id`、`transform.translation.x/y/z`（m）、`transform.rotation.x/y/z/w`（クォータニオン） |
| `/zed2i/joint_states` | `sensor_msgs/msg/JointState` | `header`、`name[]`、`position[]`（radまたはm）、`velocity[]`（rad/sまたはm/s）、`effort[]`（N·mまたはN）。未提供の配列は空の場合がある |
| `/zed2i/zed2i_description` | `std_msgs/msg/String` | `data` にURDFのXML文字列 |

現在のlaunchでは `publish_tf`、`publish_map_tf`、`publish_urdf` が有効。TFは概念的に `map → odom → camera_link` とカメラ内部の固定フレームを結ぶ。実際のカメラフレーム名にはカメラ名などが付くため、受信値で確認する。

### 6.6 実機で型・フィールド・値を確認する方法

Hub起動済みのROS 2環境で実行する。

```bash
# 配信トピックと型の一覧
ros2 topic list -t

# 型を確認し、その型の全フィールドを表示
ros2 topic type /zed2i/zed_node/odom
ros2 interface show nav_msgs/msg/Odometry

# フレーム・時刻・位置・速度をまとめて1メッセージ確認
ros2 topic echo /zed2i/zed_node/odom --once

# 速度部分のみを連続表示
ros2 topic echo /zed2i/zed_node/odom --field twist.twist

# 位置と加速度（必要に応じて別ターミナルで実行）
ros2 topic echo /zed2i/zed_node/odom --field pose.pose.position
ros2 topic echo /zed2i/zed_node/imu/data --field linear_acceleration

# 受信周期
ros2 topic hz /zed2i/zed_node/odom

# wrapper固有の型は、まず実際の型名を取得
ros2 topic type /zed2i/zed_node/pose/status
# 上の出力が zed_msgs/msg/PosTrackStatus の場合
ros2 interface show zed_msgs/msg/PosTrackStatus
```

連続表示は Ctrl+C で終了する。`--field` は表示範囲を絞るだけで、元メッセージの構造は変わらない。周期は設定値と実測値を区別し、更新停止の切り分けでは `header.stamp` も確認する。

## 7. Hub単体の確認項目

| 項目 | 確認内容 |
| --- | --- |
| 起動 | 公式wrapperがカメラを開き、起動時エラーがない |
| 実行設定 | `ros2 param dump /zed2i/zed_node` で読み込まれた設定を確認する（ノード名を変更した場合は読み替える） |
| 型 | `ros2 topic list -t` と `ros2 interface show` で6章の想定と実際を照合する |
| 画像 | 画像受信、実解像度、encoding、CameraInfoとの対応を確認する |
| センサー | IMU・磁場の時刻更新、軸、単位、欠損を確認する |
| 位置・速度 | 移動時と停止時のodom / pose、時刻、追跡状態を記録し、実移動との整合を見る |
| TF | 親子フレーム名と変換を確認し、同じ変換の重複配信がないことを確認する |
| 周期 | `ros2 topic hz` で受信周期を計測し、設定値と区別して記録する |
| 無効出力 | 深度・点群などの無効にした出力が配信されていないことを確認する |

## 8. 異常時の確認と未確認事項

| 症状 | 確認箇所 |
| --- | --- |
| カメラが開かない | 接続、デバイス指定、SDK環境、他プロセスによるカメラ使用、wrapperログ |
| トピックを受信できない | ノード起動、namespace、配信設定、ROS通信環境、publisher/subscriberのQoS |
| 値が更新されない | メッセージ受信の継続、`header.stamp`、追跡状態、wrapperログ |
| 位置・速度が不自然 | 座標系、追跡状態、停止時の変動、時刻、実移動との比較 |
| 状態情報のフィールドが表と異なる | 使用中の `zed_msgs` バージョンと `ros2 interface show` の結果 |

このリポジトリには公式wrapper本体を含まないため、Hub側の設定確認と実機の出力確認を区別する。使用中のwrapper・SDKバージョン、各トピックの実際の型、速度・共分散の設定方法、IMU加速度の重力成分の扱い、実測配信周期は実機環境で確認して記録する。
