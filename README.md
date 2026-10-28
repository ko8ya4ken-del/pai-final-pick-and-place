# 模倣学習によるロボットアームの Pick and Place

Physical AI講座2026 応用編1 最終課題　提出コード一式

ロボットアーム **CRANE+ V2** に、机上の5cm木製ブロックを掴んで別の位置へ運ばせる。MoveIt の公式サンプル `pick_and_place` の動作を記録・再生して教師データを自動生成し、**ACT（Action Chunking with Transformers）** で模倣学習させて Gazebo 上にデプロイする。

学習ステップ数を **200 / 1000 / 10000** の3水準で比較した。

## 結果

| | 200 | 1000 | 10000 |
| --- | --- | --- | --- |
| 最終 loss | 3.0 | 1.596 | **0.069** |
| Mean L2 error（ep0・559フレーム） | 0.7098 | 0.5071 | **0.3542** |
| アームの挙動 | 完全に静止 | 動くが旋回しない | **軌道を完走** |
| ブロックの移動距離 | 0 cm | 0 cm | **18.2 cm** |

10000ステップで、ブロックに到達・接触し18.2cm移動させた。ただし**把持は完全ではなく、教師データの目標位置には届いていない**（部分的な達成）。

詳細は `説明資料_PAI最終課題_ko8ya4ken.pdf` を参照。

## ベースと改変の範囲

本リポジトリは講座配布の [matsuolab/pai_training](https://github.com/matsuolab/pai_training)（第5回演習）をベースにしている。**自作ファイル・改変ファイル・未改変ファイルの区分は以下のとおり。**

| ファイル | 由来 | 内容 |
| --- | --- | --- |
| `record_states.py` | **自作** | MoveIt のデモ軌道を `/joint_states` から CSV に記録 |
| `replay_demo.py` | **自作** | CSV を再生し、エピソードごとに rosbag を収録（全自動） |
| `mon.py` | **自作** | 関節の可動域を一定時間測定（閉ループ検証用） |
| `scripts/infer.py` | **配布物を改変** | publish 周期を 20Hz → 10Hz に変更 |
| `configs/train_config.yaml` | **配布物を改変** | `delta_timestamps` ・学習ステップ数・出力先 |
| `configs/conversion_config.yaml` | **配布物を改変** | bag とデータセットのパス |
| `configs/crane_features.py` | **配布物を改変** | `repo_id` / `task_name` |
| `scripts/train.py` ほか | 配布物（未改変） | 学習・変換・評価 |
| `pyproject.toml` | 配布物（未改変） | 依存関係と pixi タスク定義 |

改変したファイルは**改変前の原本も併せて収録**している（`scripts/infer.py.bak`、`configs/train_config.yaml.orig`）。差分がそのまま確認できる。

## 工夫した点

### 1. データ収集の自動化

演習の標準手順では、キーボードによるテレオペでデモを人手収集する。これを完全に自動化した。

```
① record_states.py … MoveIt の pick_and_place 実行中の /joint_states を
                      20Hz で CSV に記録（561行・29秒分）
② replay_demo.py   … その CSV を joint_trajectory へ流し直して再生し、
                      エピソードごとに rosbag を収録（15エピソードを全自動）
```

第5回講義の「データ収集のコツ：軌道を一意にする（ACT は多峰性が苦手）」への直接の回答になっている。人がキーボードで操作すると軌道は必ずばらつくが、記録・再生方式では原理的に生じない。

### 2. 学習ステップ数の3水準比較

オフライン指標（L2誤差）と閉ループ性能（関節可動域・ブロック移動距離）の**2軸**で評価した。

---

## 再現手順

### 前提

講座配布の Docker イメージを使用する。

```bash
docker pull airobotbook/ros2-desktop-ai-robot-book-humble:latest
docker run -d --name pai_ros2 -p 6080:80 -p 9090:9090 -p 9876:9876 \
  --shm-size=512m airobotbook/ros2-desktop-ai-robot-book-humble:latest
```

ブラウザで `localhost:6080` を開き、コンテナ内で作業する。

### 1. データ収集

MoveIt のサンプルを起動し、その動作を記録する。

```bash
# Gazebo + MoveIt
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py

# 別ターミナル：記録しながらサンプルを実行
python3 record_states.py                      # /tmp/demo.csv に記録
ros2 launch crane_plus_examples example.launch.py \
     example:='pick_and_place' use_sim_time:=true
```

記録した軌道を再生して rosbag を収録する（15エピソード・全自動）。

```bash
python3 replay_demo.py --episodes 15 --outdir ~/dataset/rosbags/pick_place
```

> **`ros2 bag record` には `--use-sim-time` が必須。** 付けないと bag のタイムスタンプが実時間基準になり、変換時にフレーム数が水増しされる（`replay_demo.py` 内で指定済み）。

### 2. 変換と学習

```bash
cd ~/pai_training
pixi run convert                      # rosbag → LeRobot 形式
WANDB_MODE=offline pixi run train     # 学習（training_steps は train_config.yaml）
```

### 3. 評価（オフライン）

```bash
pixi run debug-policy \
  --dataset-root ~/dataset/lerobot_dataset/crane_plus_pick_place \
  --repo-id crane_plus_pick_place \
  --policy-path ~/checkpoints/train/crane_plus_pick_place_act_10000 \
  --episode-index 0
```

### 4. デプロイ（閉ループ）

```bash
# MoveIt なしで Gazebo を起動
ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py
```

**推論の前に、アームを初期姿勢（全関節ゼロ）へ戻す。** 学習データがこの姿勢から始まっているため、別の姿勢から始めると分布外になって方策が破綻する。

```bash
ros2 topic pub -1 /crane_plus_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [crane_plus_joint1, crane_plus_joint2, crane_plus_joint3, crane_plus_joint4],
    points: [{positions: [0.0,0.0,0.0,0.0], time_from_start: {sec: 4}}]}'

pixi run infer --model-path ~/checkpoints/train/crane_plus_pick_place_act_10000
```

関節の可動域を確認する場合：

```bash
python3 mon.py     # 60秒間測定
```

---

## つまずいた点と対処

実装・デプロイで踏んだ問題を記録しておく。**いずれもオフライン評価では検出できず、方策を実際に動かして初めて現れた。**

| | 問題 | 対処 |
| --- | --- | --- |
| 1 | 初期姿勢が学習データと異なり、分布外から始まって方策が破綻する | 推論前に全関節をゼロ姿勢へ送る |
| 2 | `n_action_steps: 1` だと方策が固定点に落ちて完全停止する | チェックポイントの `config.json` で 16（= `chunk_size`）に変更 |
| 3 | 20Hz で publish すると軌道が倍速になり、アームが追従できない | `infer.py` の timer を 0.1秒（10Hz）に変更 |
| 4 | `ros2 bag record` に `--use-sim-time` がないとフレーム数が水増しされる | 収録時に指定 |
| 5 | `/joint_states` の並びが joint1, joint2, **joint4, joint3**, hand の順 | 送信側の並びを揃える（変換スクリプトはインデックス順で読む） |

**2 について：** `n_action_steps: 1` は毎制御ステップごとに現在の関節角から推論をやり直す設定である。ACT は過去を持たないため、

```
状態 S → 行動 A を出力 → ロボットが A へ移動 → 状態が A になる → また A を出力
```

という自己一致点にはまり、動かなくなる。

**3 について：** `delta_timestamps.action` が `[0.0, 0.1, …, 1.5]` であるとおり、行動チャンクの16点は **0.1秒間隔**で学習されている。20Hz（0.05秒間隔）で publish すると軌道を倍速で再生することになり、アームが追従できず、方策は「まだ軌道の前半にいる」状態を観測して前半を出し直す。これが無限ループの原因だった。

---

## 環境

| | |
| --- | --- |
| ベースイメージ | `airobotbook/ros2-desktop-ai-robot-book-humble` |
| ROS | ROS 2 Humble |
| シミュレータ | Gazebo（Ignition Fortress） |
| パッケージ管理 | pixi |
| 実行環境 | Apple Silicon Mac（Docker・CPU学習、GPU不使用） |
