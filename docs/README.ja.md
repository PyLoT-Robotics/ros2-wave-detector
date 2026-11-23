# ROS 2 Wave Detector

[English](../README.md) · 日本語

追跡された人物のCOCO17姿勢から、肩より上の手の反復運動を検出します。
Restaurantタスクの手振り判定を、首振り・タスク進行・機体固有メッセージから分離しました。
元のpixel幅の変化だけでなく、人物幅で正規化した振幅と方向反転回数を使います。

## 姿勢JSONからの検出

COCO17のkeypointsは17×3の `[x_px,y_px,confidence]`、bboxはxyxyです。
人物ごとの安定したtrack_idを必要とし、フレーム内の行番号はIDに使用しません。
肩の中心からの相対運動を使い、静止・一方向だけの動き・下がった手・低信頼度を
除外します。時間窓・最大観測間隔・振幅・方向反転回数を設定できます。

`/wave_detector/poses` に `std_msgs/String` のJSONを入力します。

```json
{"stamp":123.4,"frame_id":"camera_color_optical_frame","people":[{"track_id":1,"bbox":[0,0,100,200],"keypoints":"17x3 numeric array"}]}
```

上のkeypointsの説明文字列は実行時には数値配列に置き換えます。
出力 `/wave_detector/events` はtrack_id、手の左右、振幅、反転回数、bbox、stampを含むJSONです。
`~/camera_moving` (`Bool`) がtrueなら履歴をクリアして判定しません。
stampは入力フレームのROS時刻です。古い入力・未来入力を除外します。

任意でpeople内に `depth_m`、最上位に `intrinsics: {fx,fy,cx,cy}` を与えると、
`/wave_detector/person_point` (`PointStamped`) にbbox中心の3D位置をmで出します。
座標はカメラ光学座標（X右、Y下、Z前）です。map座標へ変換する場合はTFを利用してください。

## YOLOカメラアダプタ（任意）

自分で用意したローカルのYOLO pose重みを `model_path` に指定すると、Image入力から
追跡付き姿勢を生成します。既存ファイルを必須とし、このコードは重みをダウンロードしません。

```bash
python -m pip install '.[yolo]'
ros2 run wave_detector wave_detector_node --ros-args \
  -p model_path:=/absolute/yolov8n-pose.pt -p device:=cpu
```

画像は `/camera/color/image_raw`、深度は `/camera/aligned_depth_to_color/image_raw`、
内部パラメータは `/camera/color/camera_info` が既定です。aligned depthのstampが
RGBの0.1秒以内で、frame_idと寸法が一致する場合のみ深度を使います。
16UC1はmm、32FC1はmです。人物bbox中央の前景深度の中央値を使います。
深度が無い場合は手振りイベントだけを出します。

Ultralyticsとモデルは別の依存です。YOLO追跡の追加依存も事前に準備してください。
実モデル推論の精度・実カメラでの動作は今回未検証です。入力遅延に合わせて
`max_age_sec` を調整してください。モデルなしのJSON経路はROS結合テストで確認します。

## オフライン再生

```bash
ros2-wave-detector replay tracked_poses.jsonl
```

1行に1フレームのstamp/peopleを保存します。`python -m wave_detector.cli demo` は
実モデルなしで反復運動を検出します。手振りはルールベースであり、動作認識モデルの
性能や呼び出し意図を保証するものではありません。カメラの移動状態は外部から入力します。

## インストールと検証

Python 3.10+。リポジトリのルートで実行します。通常のPython環境ではROSは不要です。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m wave_detector.cli demo
```

ROS 2 Jazzyで使う場合は通常のワークスペースのsrcに配置し、ROS環境でビルドします。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select wave_detector
source install/setup.bash
```

`config/params.yaml` を自分の入出力に合わせて変更してください。パラメータは起動時設定です。
`ros2 launch wave_detector wave_detector.launch.py params_file:=/absolute/path/to/params.yaml` で起動できます。
CIはPython 3.10/3.12でコアテストと合成デモを実行します。ROS結合テストはROS環境で
実行し、それ以外ではスキップします。モデル・実センサ・実アームの性能は認証しません。

## ライセンスと出典

Apache-2.0。元のコミット・ファイルと変更範囲は
[PROVENANCE.md](../PROVENANCE.md) を参照してください。元モノレポへの依存はありません。
