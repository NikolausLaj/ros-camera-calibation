import cv2
import os
import argparse

# Argument parser setup
parser = argparse.ArgumentParser(description="Capture images of a checkerboard for camera calibration.")
parser.add_argument("--num_images", type=int, default=10, help="Number of images to capture.")
parser.add_argument("--save_directory", type=str, default="calibration_images", help="Directory to save the images.")
parser.add_argument("--image_prefix", type=str, default="checkerboard_image_", help="Prefix for saved images.")
args = parser.parse_args()

# Create save directory if it doesn't exist
if not os.path.exists(args.save_directory):
    os.makedirs(args.save_directory)

# Open webcam
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("Error: Could not open the webcam.")
    exit()

print("Press 'c' to capture an image or 'q' to quit.")

count = 0
while count < args.num_images:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Show the frame
    flip_frame = cv2.flip(frame, 1)
    cv2.imshow("Webcam", flip_frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        # Capture and save the image
        image_path = os.path.join(args.save_directory, f"{args.image_prefix}{count+1}.jpg")
        cv2.imwrite(image_path, frame)
        print(f"Captured image {count+1} at {image_path}")
        count += 1
    elif key == ord('q'):
        print("Quitting...")
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
print("Image capture complete.")
