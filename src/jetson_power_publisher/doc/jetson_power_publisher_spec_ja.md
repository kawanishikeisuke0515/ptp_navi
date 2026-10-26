# Jetson Power Publisher 仕様書

Version: 0.1 draft  
Date: 2026-09-18  
Status: 初期実装済み。実機出力・負荷・権限の確認待ち。

## 1. 目的と対象

Jetson Orin Nanoのモジュール消費電力をROS 2トピックとして配信し、ドッキング実験の位置・姿勢・制御指令とともに記録する。

対象環境はJetson Orin Nano、ユーザー申告のJetson Linux「R36、REVISION 4.7」。JetPackのバージョンは推定せず、実装前に `/etc/nv_tegra_release` と実機の `tegrastats` 出力を確認する。

計測対象は `tegrastats` の `VDD_IN`。これは電源系統の名前であり、表示値は電圧ではなく、その系統の現在電力・平均電力である。NVIDIA資料ではOrin NanoのVDD_INをTotal Module Powerと定義している。

キャリアボード、外部機器、モーター、電源変換損失を含むロボット全体の計測値とは扱わない。各電源レールは計測範囲が重複し得るため合算しない。

対象と計測方法は合意済み。以下の型名、パラメータ、異常時動作は初期実装の仕様案とする。

## 2. 構成と責務

```text
tegrastats --interval 1000
  └─ 標準出力 → jetson_power_publisher → /jetson/power
                                          └─ docking_logger
                                               ├─ jetson_power.csv
                                               └─ timeline.csv
```

- `jetson_power_publisher`：独立したROS 2 Pythonパッケージ・同名ノード。子プロセス管理、行の解析、単位変換、有効性判定、Publishを担当する。
- `jetson_interfaces`：新規のROS 2インターフェースパッケージ。`JetsonPower.msg` を定義する。
- `docking_logger`：受信値を変換・再計算せず既存の非同期CSVライターで保存する。
- 電力量[Wh]の積算は初版ではログの後処理とする。欠測区間は無条件に補間しない。
- CPU/GPU使用率、温度、レール別詳細、電力制限の変更は初版の対象外。

ノード、設定YAML、単体launch、テストと `jetson_interfaces` を配置済み。

## 3. ROSインターフェース

トピック：`/jetson/power`  
型：`jetson_interfaces/msg/JetsonPower`  
QoS：Reliable / Volatile / Keep Last、depth=10。

```text
std_msgs/Header header
string total_power_rail
float64 power_w
float64 average_power_w
bool valid
string status
string raw_line
```

| フィールド | 定義 |
|---|---|
| header.stamp | 出力行をノードが受信したROS時刻。ハードウェア計測時刻ではない |
| header.frame_id | 空文字。空間座標系を持たないため |
| total_power_rail | 対象レール名。初期値 `VDD_IN` |
| power_w | tegrastatsが報告した現在電力をWに変換した値 |
| average_power_w | tegrastatsが報告した平均電力をWに変換した値 |
| valid | 現在電力・平均電力の両方が解析でき、有限かつ0以上ならtrue |
| status | `ok` / `parse_error` / `rail_missing` / `timeout` / `process_exited` |
| raw_line | 解析対象の元の1行。末尾改行のみ除去 |

平均値の集計期間は実機で未確認。直近1秒平均、実験全体平均、ノード独自の移動平均とは表記しない。子プロセスの再起動前後で平均値の連続性を仮定しない。

## 4. 取得と解析

1. ノード起動時に `tegrastats` を1プロセス起動し、継続して標準出力を読み取る。周期ごとのプロセス起動は行わない。
2. 出力行を受信した時点でROS時刻と単調増加時計を取得する。
3. 設定されたレール名と完全一致する電力項目を抽出する。他のレールや温度・使用率から代用しない。
4. `現在値/平均値` をmWとして読み、各値を1000で割ってWに変換する。
5. 通常は出力行ごとに1メッセージをPublishする。同じ測定値をタイマーで繰り返し正常値として送信しない。

対応予定の表記例（実機出力ではなく解析仕様の例）：

```text
VDD_IN 5000mW/4500mW
VDD_IN 5000/4500
```

整数・小数、単位付き・単位なしの上記形式に対応する。未知の単位、片側欠落、非有限値、負値、対象レールの重複は解析エラーとする。対象レール自体がない場合は `rail_missing`。0 Wは欠測を意味しない。

## 5. 異常時とプロセス管理

- 解析失敗・対象レール欠落：当該行に対して `valid=false`、両電力値をNaN、元の行とエラーstatusを配信する。
- 起動後または最後の行受信から `output_timeout_sec` を超えて行が届かない：`timeout` を配信する。判定には単調増加時計を使う。
- 子プロセス終了：`process_exited` を配信する。初版は自動再起動しない。ノードは異常状態を配信し続け、復旧にはノード再起動を必要とする。
- 行を伴わない異常通知のheader.stampは通知時刻、raw_lineは空文字とする。両電力値はNaN、validはfalse。
- 異常検知時に1回通知し、無出力・終了状態が続く間は `interval_ms` 相当の周期で異常通知を続ける。出力が再開したtimeout状態は次の行の結果で復帰する。
- 実行ファイルなし・起動権限不足・不正パラメータは起動失敗として理由を出力し終了する。ロガー側は未受信状態になる。
- 標準出力待ちでROSコールバックを停止させない。専用読取処理を用い、stderrも排出してパイプ詰まりを防ぐ。警告は頻度制限する。
- shellを介さず引数リストで起動する。ノード内でsudoを実行しない。
- 終了時には自分が生成したプロセスだけをterminateし、2秒以内に終了しなければkillして回収する。他のtegrastatsも停止させる `--stop` や `pkill` は使わない。
- 初版は実時間の実機計測を対象とし、`use_sim_time=true` は起動時エラーとする。

## 6. パラメータ

全パラメータは起動時に確定し、変更時はノードを再起動する。

| 名前 | 初期値 | 条件・意味 |
|---|---|---|
| tegrastats_path | `tegrastats` | PATHで解決する実行名、または実行ファイルのパス。空文字不可 |
| power_topic | `/jetson/power` | Publish先。空文字不可 |
| total_power_rail | `VDD_IN` | 完全一致で抽出するレール名。空文字・空白を含む名前は不可 |
| interval_ms | `1000` | 正の整数。tegrastatsの出力間隔[ms] |
| output_timeout_sec | `3.0` | 有限で取得間隔より大きい値。起動直後も同じ閾値 |

1秒周期と3秒のtimeoutは初期値であり、実測した出力間隔・処理負荷を見て評価する。周期短縮がセンサー自体の更新速度向上を保証するわけではない。

## 7. docking_logger連携

ストリーム名を `jetson_power`、購読パラメータを `jetson_power_topic`（初期値 `/jetson/power`）とする。既存のQoS・鮮度設定方式に従う。

`jetson_power.csv` の列：

```text
sample_seq, recv_ros_time_ns, elapsed_sec, source_stamp_ns, frame_id,
total_power_rail, power_w, average_power_w, valid, status, raw_line
```

- 共通列の時刻・シーケンス定義、NaNのCSV表現、書き込み失敗時の処理は既存ロガーに従う。
- `timeline.csv` に上記列とreceived / age_sec / stale / value_validを `jetson_power_` 接頭辞で追加する。
- `value_valid` はmsg.valid、両電力値の有限性・非負性で判定する。
- `jetson_power_stale_timeout_sec` の初期値は3秒とする。既存の全体初期値1秒では1 Hz取得時に境界付近でstaleになり得るため、設定YAMLで明示する。
- 異常通知の受信はageを更新する。そのためstale=falseでもvalue_valid=falseになり得る。staleはトピック受信の鮮度であり、正常計測の保証ではない。
- timelineは各ストリームの最新値を保持する。20 Hzのtimelineに1 Hzの電力値が繰り返し現れても新規計測とは扱わない。解析時はsource_stamp_nsまたはsample_seqで識別する。
- metadataに計測範囲「Jetson module / configured rail」、単位W、平均値はtegrastats報告値であることを記載する。設定スナップショットを渡す場合は取得ノードのYAMLも含める。
- schema_version=4。電力ストリームとタイムライン列を追加済み。

ロガー単体launchは計測ノードを起動しない。計測ノード専用launchを用意し、統合bringupへの追加は別途行う。計測ノード未起動でも既存ストリームの記録は続ける。

## 8. 検証・受け入れ条件

### 実機なしの検証

- 単位付き・単位なし、小数、複数レールを含む行からVDD_INだけを取得できる。
- 5000/4500を5.0/4.5 Wに変換し、0を有効として保持する。
- 欠落、重複、不正単位、負値、NaN/Infを正常値として配信しない。
- 模擬プロセスで無出力、stderr出力、途中終了、出力再開、終了時の回収を確認する。
- ROSメッセージの時刻・status・NaNと、CSV・timeline・metadataへの保存を確認する。
- 未受信、正常、異常通知受信、トピック停止をreceived / value_valid / staleで区別できる。

### 実機での確認事項

- `/etc/nv_tegra_release`、Orin Nanoの機種情報、`tegrastats --interval 1000` の数行を採取する。
- VDD_INの存在、単位表記、一般ユーザーでの実行可否、出力間隔を確認する。
- アイドル時と処理負荷を掛けた時で、元出力とトピックの値・単位が一致することを確認する。
- docking_loggerと同時実行してCSVを取得し、制御処理への影響と欠測を確認する。
- ノード停止後に子プロセスが残らず、他のtegrastatsプロセスを停止しないことを確認する。

## 9. 参照資料

- [NVIDIA：Orin Nano / NXの電力監視とVDD_INの計測範囲（R36.4）](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)
- [NVIDIA：Tegrastats Utility、電力表示とinterval（R36.4.3）](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html)
- [既存docking_logger仕様書](../../docking_logger/doc/docking_logger_spec_ja.md)

参照資料はR36系の公開資料。申告されたREVISION 4.7での具体的な出力形式は、上記の実機確認で確定する。

## 10. ビルドと起動

ROS 2環境をsourceしたワークスペースで実行する。

```bash
colcon build --packages-up-to jetson_power_publisher docking_logger
source install/setup.bash
ros2 launch jetson_power_publisher jetson_power_publisher.launch.py
```

別端末でも同じ環境をsourceして確認・記録する。

```bash
ros2 topic echo /jetson/power
ros2 launch docking_logger docking_logger.launch.py target_marker_id:=7
```

`target_marker_id` は実験対象に合わせる。取得周期などは設定YAMLを変更し、launchの `config:=/path/to/config.yaml` で指定できる。tegrastatsはJetson実機側にインストールされているものを使用する。
