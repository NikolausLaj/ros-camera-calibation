# Camera Calibration for ROS 2

This ROS 2 package provides tools for camera calibration using OpenCV. The package includes nodes for capturing images of a checkerboard pattern and performing calibration to obtain intrinsic camera parameters. These parameters are then saved to a YAML file for use in other ROS applications.

## Package Overview

- **camera_calibration.py**: The primary ROS 2 node that performs camera calibration based on captured images of a checkerboard pattern.
- **take_images.py**: A script to capture images of a checkerboard using a connected webcam.
- **camera_calibration_launch.py**: Launch file for starting the calibration node.
- **config.yaml**: Config file to set the neede parameters for the calibraion.
- **camera_parameters.yaml**: Stores the output camera matrix and distortion coefficients after calibration.

## Features

- **Capture Calibration Images**: Use `take_images.py` to capture a series of images of a checkerboard pattern.
- **Perform Camera Calibration**: Calibrate the camera to find intrinsic parameters using `camera_calibration.py`.
- **Save Calibration Results**: Save calibration parameters, including the camera matrix and distortion coefficients, in YAML format.

## Dependencies

This package depends on the following libraries:

- `rclpy` (ROS 2 Python Client Library)
- `cv2` (OpenCV)
- `numpy`
- `yaml`

Install these dependencies via ROS 2 and pip as needed.

## Setup

Clone the repository and install the package:

```bash
git clone <repository-url>
cd camera_calibration
colcon build
source install/setup.bash
```

## Usage
### 1. Capture Calibration Images

Use take_images.py to capture checkerboard images for calibration. The script will save images to a specified directory.

```bash
python3 take_images.py --num_images 10 --save_directory ./calibration_images --image_prefix checkerboard_image_
```
- --num_images: Number of images to capture.
- --save_directory: Directory to save images.
- --image_prefix: Prefix for image filenames.

**Press 'c' to capture an image or 'q' to quit.**

### 2. Perform Camera Calibration

Once images are captured, use the ROS 2 node camera_calibration.py to perform calibration.

Edit camera_parameters.yaml with your checkerboard dimensions, square size, and paths.
Launch the calibration node:

```bash
ros2 launch camera_calibration camera_calibration_launch.py
```

## Calibration Parameters

In config.yaml, several parameters can be set:

```yaml
camera_calibration_node:
  ros__parameters:
    checkerboard_dims: [7, 9]
    square_size: 2.0
    image_path: '/path/to/images'
    output_path: '/path/were/output/should/be/saved'
```

The calibrated parameters will be logged and saved to camera_parameters.yaml:

```yaml
camera_parameters:
  ros__parameters:
    camera_matrix:
      cols: 3
      data: [...]
      rows: 3
    distortion_coefficients:
      cols: 5
      data: [...]
      rows: 1
```
