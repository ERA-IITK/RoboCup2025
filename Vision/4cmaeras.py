import cv2
import threading
import time
import numpy as np
from ultralytics import YOLO


model = YOLO("yolo11n.pt")

# Export the model to TensorRT (creates 'yolo11n.engine')
model.export(format="engine")

# Load the exported TensorRT model
trt_model = YOLO("yolo11n.engine")

# ----------------------------
# Global variables for frame sharing
# ----------------------------
raw_frames = {}
processed_frames = {}
frame_lock = threading.Lock()

# List of camera indices or device paths (adjust as needed)
camera_ids = [0, 1, 2, 3]

# ----------------------------
# YOLO Inference Function
# ----------------------------
def yolo_inference(image):
 
    # Run inference using the TensorRT model
    results = trt_model(image)
    
    # Annotate the image with the detection results
    # .plot() returns the image with drawn detections.
    annotated_image = results[0].plot()  # Note: this may differ based on your model version.
    return annotated_image

# ----------------------------
# Camera Thread Function
# ----------------------------
def camera_thread(cam_id):
    """
    Continuously capture frames from a camera,
    applying custom settings such as manual exposure.
    """
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print(f"Error: Camera {cam_id} failed to open.")
        return

    # Set custom camera settings:
    # On Ubuntu, setting CAP_PROP_AUTO_EXPOSURE to 1 typically disables auto exposure.
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    # Manually set exposure value (adjust the value as per your requirements)
    cap.set(cv2.CAP_PROP_EXPOSURE, 100)

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Warning: Camera {cam_id} frame grab failed.")
            continue

        # Update the shared raw_frames dictionary with the latest frame
        with frame_lock:
            raw_frames[cam_id] = frame.copy()
        # Brief sleep to allow thread switching
        time.sleep(0.01)

# ----------------------------
# YOLO Thread Function
# ----------------------------
def yolo_thread():
    """
    Continuously process the latest frames from all cameras using YOLO inference.
    """
    while True:
        # Copy current frames to process
        with frame_lock:
            frames_to_process = {cam_id: frame.copy() for cam_id, frame in raw_frames.items()}

        # Run YOLO inference on each available frame
        for cam_id, frame in frames_to_process.items():
            processed_frame = yolo_inference(frame)
            with frame_lock:
                processed_frames[cam_id] = processed_frame

        # Small sleep to control inference rate
        time.sleep(0.01)

cam_threads = []
for cam_id in camera_ids:
    t = threading.Thread(target=camera_thread, args=(cam_id,), daemon=True)
    cam_threads.append(t)
    t.start()

t_yolo = threading.Thread(target=yolo_thread, daemon=True)
t_yolo.start()


while True:
    display_frames = {}
    with frame_lock:
        # Prefer processed frames; fallback to raw frames if necessary
        for cam_id in camera_ids:
            if cam_id in processed_frames:
                display_frames[cam_id] = processed_frames[cam_id]
            elif cam_id in raw_frames:
                display_frames[cam_id] = raw_frames[cam_id]
    
    # Display each frame in its own window
    for cam_id, frame in display_frames.items():
        window_name = f"Camera {cam_id}"
        cv2.imshow(window_name, frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
