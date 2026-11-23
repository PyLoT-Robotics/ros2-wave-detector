# ROS 2 Wave Detector

English · [日本語](docs/README.ja.md)

Detect repeated raised-hand motion from tracked COCO17 person poses.
The Restaurant task's waving logic is separated from head scanning, task flow
and robot-specific messages. Detection uses body-width-normalized amplitude
and direction reversals rather than pixel displacement alone.

## Core and JSON pose input

COCO17 keypoints are a 17×3 array of `[x_px,y_px,confidence]`; boxes use xyxy.
Each person needs a stable `track_id`, not their row index within a frame.
Motion is measured relative to the shoulder center. Static hands, one-directional
motion, lowered hands and low-confidence observations are excluded. Configure
the time window, maximum observation gap, amplitude and direction-reversal count.

Publish JSON in `std_msgs/String` on `/wave_detector/poses`:

```json
{"stamp":123.4,"frame_id":"camera_color_optical_frame","people":[{"track_id":1,"bbox":[0,0,100,200],"keypoints":"17x3 numeric array"}]}
```

Replace the explanatory keypoints string above with an actual numeric array.
`/wave_detector/events` emits JSON with the track ID, hand side, amplitude,
reversal count, bounding box and timestamp. When `~/camera_moving` (`Bool`) is
true, history is cleared and detection is suspended. Timestamps use the input
frame's ROS time; stale and future-dated input is rejected.

Optionally provide `depth_m` per person and top-level
`intrinsics: {fx,fy,cx,cy}` to publish the bounding-box center's 3D position in
metres on `/wave_detector/person_point` (`PointStamped`). Coordinates use the
camera optical frame: X right, Y down, Z forward. Use TF for map-frame output.

## Optional YOLO camera adapter

Supply a local YOLO pose weights file through `model_path` to produce tracked
poses from Image input. An existing file is required; this code does not download
weights.

```bash
python -m pip install '.[yolo]'
ros2 run wave_detector wave_detector_node --ros-args \
  -p model_path:=/absolute/yolov8n-pose.pt -p device:=cpu
```

Default inputs are `/camera/color/image_raw`,
`/camera/aligned_depth_to_color/image_raw` and `/camera/color/camera_info`.
Aligned depth is used only when its timestamp is within 0.1 seconds of RGB and
frame IDs and image dimensions match. 16UC1 depth is interpreted as millimetres;
32FC1 is metres. The node uses a foreground depth median in the central person
bounding box. Without depth, it publishes waving events only.

Ultralytics and models are external dependencies. Prepare any additional YOLO
tracking dependencies in advance. Real-model accuracy and camera operation have
not been validated here. Adjust `max_age_sec` for your input/inference latency.
The model-free JSON path is covered by a ROS integration test.

## Offline replay

```bash
ros2-wave-detector replay tracked_poses.jsonl
```

Save one frame's stamp/people per JSONL line. `python -m wave_detector.cli demo`
detects synthetic repeated motion without a model. This is a rule-based detector;
it does not certify action-recognition accuracy or a person's intent to call
the robot. Camera motion state must be supplied externally.

## Installation and checks

Requires Python 3.10+. Run these commands from the repository root.
The core works in a regular Python environment without ROS.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m wave_detector.cli demo
```

For ROS 2 Jazzy, place the repository in a regular workspace's `src` directory
and build it in a ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select wave_detector
source install/setup.bash
```

Adapt `config/params.yaml` to your inputs and outputs. Parameters are read at startup.
Start the node with
`ros2 launch wave_detector wave_detector.launch.py params_file:=/absolute/path/to/params.yaml`.
CI runs core tests and the synthetic demo on Python 3.10 and 3.12. ROS integration
tests run when ROS is available and are skipped otherwise. These checks do not
certify model, physical-sensor or real-arm performance.

## License and source

This repository is currently private. Apache-2.0. See
[PROVENANCE.md](PROVENANCE.md) for the source commit, files and extraction changes.
There is no dependency on the original monorepo.
