import cv2
import numpy as np

# Load your distorted image
distorted_image = cv2.imread('/ros-humble-dev-container/src/camera_calibation/calibration_images/calib_img_2.jpg')

fx = 603.3565980559212
fy = 603.8525136324047
cx = 314.98325947311577
cy = 233.97939039940036 

k1 =  0.07904876200503386
k2 = -0.2473724352661824
p1 =  1.4961220610951097e-05
p2 = -0.0024014898368810893
k3 = -0.2125031132173052

 

# Define your camera matrix and distortion coefficients
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])
dist_coeffs = np.array([k1, k2, p1, p2, k3])  # Example format

# Undistort the image
undistorted_image = cv2.undistort(distorted_image, camera_matrix, dist_coeffs)

# Display the result
cv2.imshow("Undistorted Image", undistorted_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
