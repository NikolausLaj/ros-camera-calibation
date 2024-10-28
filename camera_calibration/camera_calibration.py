import numpy as np
import cv2
import glob
import rclpy
from rclpy.node import Node
import os
import sys

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # Declare parameters and get them from config
        self.declare_parameter('checkerboard_dims', [7, 7])
        self.declare_parameter('square_size', 1.0)
        self.declare_parameter('image_path', '')

        # Retrieve the parameters
        checkerboard_dims = self.get_parameter('checkerboard_dims').get_parameter_value().integer_array_value
        self.checkerboard_size = (checkerboard_dims[0], checkerboard_dims[1])
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value

        self.get_logger().info(f"Checkerbord dimensions: {self.checkerboard_size}")
        self.get_logger().info(f"Square size: {self.square_size}")
        self.get_logger().info(f"Image path: {self.image_path}")

        # Verify the image path
        if not os.path.exists(self.image_path):
            self.get_logger().error(f"The specified image path does not exist: {self.image_path}")
            sys.exit(1)

        # Perform the calibration
        self.perform_calibration()

    def perform_calibration(self):
        # Define the criteria for corner sub-pixel refinement
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points for the checkerboard pattern
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        object_points = []  # 3D points in real-world space
        image_points = []   # 2D points in image plane

        # Load images from the specified path
        images = glob.glob(f'{self.image_path}/*.jpg')
        if not images:
            self.get_logger().error(f"No images found in the specified path: {self.image_path}")
            sys.exit(1)

        # Process each image
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the checkerboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
            if ret:
                object_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                image_points.append(corners2)

                # Optional: display the corners
                cv2.drawChessboardCorners(img, self.checkerboard_size, corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(100)

        cv2.destroyAllWindows()

        # Perform camera calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, gray.shape[::-1], None, None
        )

        # Log calibration results
        self.get_logger().info(f"Camera Matrix:\n{camera_matrix}")
        self.get_logger().info(f"Distortion Coefficients:\n{dist_coeffs}")

        # Save calibration data
        np.savez('calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)
        self.get_logger().info("Calibration data saved to 'calibration_data.npz'")

def main(args=None):
    rclpy.init(args=args)
    camera_calibration_node = CameraCalibrationNode()
    rclpy.spin(camera_calibration_node)
    camera_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
