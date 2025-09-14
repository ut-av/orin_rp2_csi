# orin_rp2_csi

This ROS2 package provides nodes for processing images from NVIDIA Jetson Orin RP2 CSI cameras, supporting both mono and stereo camera configurations.

## Features

- Mono camera processing with image rectification and publishing.
- Stereo camera processing with disparity computation.
- Launch files for easy startup with configurable parameters.

## Build Instructions

1. Clone or place this package in your ROS2 workspace: `src/orin_rp2_csi`
2. Build the workspace:
   ```bash
   colcon build --packages-select orin_rp2_csi
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Mono Camera

Launch the mono processor for camera 0 without display:
```bash
ros2 launch orin_rp2_csi mono.launch.py sensor_id:=0 display_mode:=none
```

Launch the mono processor for camera 1 with image display:
```bash
ros2 launch orin_rp2_csi mono.launch.py sensor_id:=1 display_mode:=image
```

### Stereo Camera

Launch the stereo processor with side-by-side display:
```bash
ros2 launch orin_rp2_csi stereo.launch.py display_mode:=side_by_side
```

## Parameters

- `sensor_id`: Selects the camera sensor (0 or 1) for mono mode.
- `display_mode`: Controls display output ('none', 'image' for mono, 'side_by_side' for stereo).

## Topics

- `/image`: Published rectified mono image.
- `/disparity`: Published disparity image from stereo processing.