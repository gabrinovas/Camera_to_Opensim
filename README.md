# Camera to OpenSim ROS2 Packages

This repository contains two ROS2 Humble packages designed to bridge the gap between RealSense cameras (or generic RGB cameras) and OpenSim musculoskeletal modeling software.

## Packages

### 1. `camera_to_opensim`
A Python-based package containing the core logic nodes:
*   **`pose_estimation_node`**: Uses `rtmlib` to detect 2D keypoints from RGB images (`/camera/color/image_raw`). Publishes `camera_to_opensim_interfaces/PersonArray`.
    *   *Note*: If `rtmlib` is not installed, it runs in **MOCK MODE**, publishing dummy keypoints for testing.
*   **`data_processing_node`**: Subscribes to 2D keypoints and aligned depth images (`/camera/aligned_depth_to_color/image_raw`).
    *   Reconstructs specific 3D points using the Pinhole Camera Model and depth data.
    *   Records data to memory.
    *   On stop, saves a `.trc` file and triggers `Pose2Sim` Inverse Kinematics to generate `.mot` files.
*   **`sports2d_node`**: A fallback node used when RealSense depth is disabled.
    *   Records RGB video.
    *   On stop, processes the video using `Sports2D` (which includes its own depth estimation model) to generate OpenSim files.

### 2. `camera_to_opensim_interfaces`
A CMake-based package defining custom ROS2 messages:
*   `Keypoint2D`: `string name`, `float32 x`, `float32 y`, `float32 confidence`
*   `Person`: `int32 id`, `Keypoint2D[] keypoints`
*   `PersonArray`: `std_msgs/Header header`, `Person[] persons`

## Dependencies

*   **ROS2 Humble** (Desktop Full recommended)
*   **RealSense ROS2 Wrapper** (`realsense2_camera` package)
*   **Python Libraries**:
    *   `rtmlib` (for 2D Pose Estimation) -> `pip install rtmlib`
    *   `Pose2Sim` (for Inverse Kinematics) -> Ensure it is in your `PYTHONPATH`
    *   `Sports2D` (for Fallback) -> Ensure it is in your `PYTHONPATH`
    *   `numpy`, `opencv-python`

## Installation

1.  Clone this repository into your ROS2 workspace `src` folder.
2.  Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  Build the packages:
    ```bash
    colcon build --packages-select camera_to_opensim camera_to_opensim_interfaces
    ```
4.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Usage

### Launching the System

The main launch file is `convert.launch.py` in the `camera_to_opensim` package.

#### Option A: Use RealSense Depth (Recommended)
This uses the RealSense depth stream for 3D reconstruction.
```bash
ros2 launch camera_to_opensim convert.launch.py use_realsense_depth:=true launch_realsense:=true
```

#### Option B: Use Sports2D Fallback
This uses `Sports2D`'s model-based depth estimation (no depth camera required).
```bash
ros2 launch camera_to_opensim convert.launch.py use_realsense_depth:=false
```

### Recording Data

The system uses ROS2 services to control data recording.

1.  **Start Recording**:
    ```bash
    ros2 service call /start_recording std_srvs/srv/Trigger
    ```

2.  **Stop Recording**:
    *   In **RealSense Mode**: This saves the `.trc` file and automatically runs `Pose2Sim` IK.
    *   In **Sports2D Mode**: This saves the video and automatically runs `Sports2D` processing.
    ```bash
    ros2 service call /stop_recording std_srvs/srv/Trigger
    ```

### Output

Data is saved by default to `~/opensim_data` (configurable via `output_dir` launch argument).
*   **RealSense Mode**: Look in `~/opensim_data/pose-3d/` for `.trc` and `.mot` files.
*   **Sports2D Mode**: Look in `~/opensim_data/<timestamp>/` for Sports2D output.

### Coordinate System
The `data_processing_node` automatically converts coordinates from the Camera Frame (OpenCV standard) to the OpenSim Frame (Typical Y-Up):
*   **OpenSim X** (Forward) = Camera Z (Depth)
*   **OpenSim Y** (Up) = -Camera Y (Up/Down inverted)
*   **OpenSim Z** (Right) = Camera X (Right)

## Architecture Diagram

```mermaid
graph TD
    subgraph Inputs
        RGB[RGB Image] --> PE[Pose Estimation Node]
        Depth[Depth Image] --> DP[Data Processing Node]
        RGB --> S2D[Sports2D Node]
    end

    subgraph "camera_to_opensim"
        PE -- PersonArray (2D) --> DP
        DP -- "Sync & Reconstruct" --> TRC[.trc File]
        TRC --> IK[Pose2Sim Wrapper]
        IK --> MOT[.mot File]
        
        S2D -- "Record Video" --> VID[.mp4 File]
        VID --> S2DW[Sports2D Wrapper]
        S2DW --> MOT_S2D[.mot File (Sports2D)]
    end

    subgraph Outputs
        MOT
        MOT_S2D
    end
```
