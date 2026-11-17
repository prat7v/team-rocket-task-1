# Team Rocket — Vision-Based Robotic Shape Sorting

## Selected Task

**Task 1 — ME 639: Introduction to Robotics**

## Project Description

This project develops a vision-based shape-sorting system for a DYNAMIXEL robotic arm and an Intel RealSense RGB-D camera. The camera identifies coloured objects and matching holes on a sorting board. Image processing determines object shape and position, while camera-to-robot registration and inverse kinematics support planning a pick-and-place sequence.

The supported objects are a blue square, yellow triangle, lime circle, and orange hexagon. The project includes saved-frame perception, live camera preview, robot calibration utilities, manual control, and a configurable pick-and-place executor.

## Approach and Methodology

### 1. Colour and Shape Detection

RGB images are converted to HSV colour space. Configured colour thresholds segment candidate objects, and morphological operations reduce noise. Contours are classified by geometry to distinguish the four expected shapes. The detector checks each object's observed shape against its expected colour/shape combination and matches it to a corresponding board opening.

The camera pick-and-place module also estimates the bright top face of each peg and excludes candidates overlapping detected socket rims. For live execution, the selected peg and matching slot must be unambiguous and stable across ten frames.

### 2. Position Estimation and Calibration

Aligned depth data and RealSense camera intrinsics are used to estimate camera-frame object positions. An ArUco board reference supports transformation into the robot coordinate frame. Calibration includes the camera/board relationship, robot reference measurements, tool geometry, joint encoder limits, and task heights.

Calibration settings belong to the recorded camera view and robot arrangement; they must be checked when the camera, board, or tool is repositioned.

### 3. Motion Planning

The pick-and-place module uses fixed-pitch inverse kinematics for the four-joint arm. It constructs a sampled motion sequence and checks encoder limits, inverse-kinematics continuity, and intersections between modelled robot links and configured obstacle boxes.

The planned sequence opens the gripper, approaches the peg, grasps, lifts, transfers above the matching hole, lowers, releases, and retracts. Grasp contact is inferred from a calibrated motor-current threshold.

### 4. Robot Control

Joint commands are sent through the DYNAMIXEL SDK. Manual keyboard control supports setup and joint inspection. Camera preview is the default mode of the pick-and-place program and does not access the serial motor interface. Hardware execution requires an explicit execution option and completed commissioning checks.

## Key Features

- Detection and matching of four colour/shape combinations.
- RGB-D position estimation and annotated detection images.
- Saved-frame processing for reproducible perception checks.
- Live preview with stable-match detection.
- Camera-to-robot registration using an ArUco reference.
- Fixed-pitch inverse kinematics and sampled path checks.
- Configurable grasp, travel, release, and obstacle parameters.
- Manual robot control and saved detection/planning reports.

## Technologies Used

- Python
- OpenCV (`opencv-contrib-python`)
- NumPy
- Intel RealSense SDK (`pyrealsense2`)
- DYNAMIXEL SDK (`dynamixel-sdk`)
- ArUco markers and RGB-D calibration
- Pytest for the included perception tests

## Project Files

The main components are:

| File or directory | Purpose |
| --- | --- |
| `camera_pick_place.py` | Live/offline perception, planning, and optional pick-and-place execution |
| `camera_pick_place_config.json` | Camera, geometry, task parameters, and commissioning records |
| `keyboard_teleop.py` | Robot communication and manual control |
| `joint_limit_measurements.json` | Recorded joint encoder bounds |
| `scripts/observe_shape_sorting.py` | Saved RGB-D colour/shape detection and matching |
| `scripts/capture_calibration_baseline.py` | RealSense capture utility |
| `config/shape_observation.json` | Colour thresholds, regions, and shape settings |
| `data/camera_calibration/` | Saved captures, calibration data, and detection evidence |
| `tests/test_shape_observation.py` | Perception regression tests |

Include the helper modules imported by `camera_pick_place.py`, including `live_blue_path_preview.py`, `inspect_robot_kinematics.py`, and `study_fixed_layout_tilt.py`.

## Setup

### Requirements

- Python environment compatible with the installed RealSense SDK.
- Intel RealSense RGB-D camera for live operation.
- DYNAMIXEL arm, gripper, USB interface, and power supply for motor operation.
- Saved calibration files and the matching configuration.

From the project environment, install:

```bash
python -m pip install numpy opencv-contrib-python pyrealsense2 dynamixel-sdk pytest
```

Use `opencv-contrib-python` so that the ArUco module is available. Close RealSense Viewer before opening the camera from Python.

### Source Layout

The inspected `camera_pick_place.py` currently resolves its helper project relative to its location:

```text
<parent-of-script-directory>/work/restored/Dynamixel_Robot/
```

That folder supplies `scripts/` and `config/shape_observation.json`. Keep the original layout, or update the `PROJECT` assignment and helper import paths to match the repository layout before running from a fresh clone. Keep `camera_pick_place_config.json`, `keyboard_teleop.py`, and their dependencies accessible to the entry script. The current executor selects `COM3`; adapt it to the verified hardware port for another machine.

## How to Run

### Reproduce Saved-Frame Detection

From the detection package root, with its `scripts`, `config`, and `data` directories intact:

```bash
python scripts/observe_shape_sorting.py data/camera_calibration/20261106_014158_245360
```

This processes a recorded RGB-D capture and writes a new result folder. It does not command the robot.

Run the included perception checks with:

```bash
python -m pytest tests/test_shape_observation.py -q
```

### Start Live Camera Preview

From the directory containing the camera pick-and-place entry script:

```bash
python camera_pick_place.py --colour blue
```

Available colours are `blue`, `yellow`, `lime`, and `orange`.

- Press **P** in the OpenCV window to save the annotated image and detection report.
- Press **Q** or **Esc** to exit.

### Inspect an Offline Capture

```bash
python camera_pick_place.py --colour blue --session <saved_capture_directory>
```

The capture directory must contain `color.png`, `aligned_depth_metres.npy`, and `camera_baseline.json`. Offline processing saves an annotated image and JSON report without motor execution.

### Execute a Commissioned Pick-and-Place Cycle

Verify the robot registration, encoder mapping, tool pitch, task heights, contact-current threshold, and obstacle geometry against the physical setup. Close other motor clients so the serial port is available.

After commissioning:

```bash
python camera_pick_place.py --colour blue --execute
```

This handles one selected peg per invocation and starts after a stable matching detection. Execution uses fresh camera frames; `--execute` cannot be combined with `--session`. Keep the OpenCV window focused; **Q/Esc** requests a hold during execution. Motors may remain enabled to support the arm after completion.

## Results and Current Scope

Archived detection runs identify all four expected objects and one matching hole for each shape. Three saved reports record `all_four_shapes_matched: true`, supporting the perception and matching component of the project.

The inspected camera pick-and-place module includes planning and execution logic, but its accompanying verification notes do not establish a physically commissioned autonomous insertion run. Camera/robot registration and task geometry need validation for the actual setup. Detection success alone does not establish successful grasping or insertion.

The obstacle model checks sampled link geometry against static boxes. It does not fully model carried-object contact, insertion dynamics, or all self-collisions. Shape-dependent rotational alignment must also be commissioned.

## Demo Video

**[Watch the Task 1 demonstration](REPLACE_WITH_UNLISTED_DEMO_URL)**

Replace the placeholder with the unlisted demonstration link before submission. The video must be at most **3 minutes**, show the project running, and explain its main functionality. Clearly identify which stages are demonstrated: perception, planning, manual control, or physical pick-and-place.
