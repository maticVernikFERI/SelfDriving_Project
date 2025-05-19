import cv2
from ultralytics import YOLO
import time

def run_yolo_detection():
    """Runs YOLOv8 detection on a Raspberry Pi 4 using a webcam and prints results to the terminal."""

    try:
        # Load the YOLOv8 model. Replace 'yolov8n.pt' with your desired model.
        model = YOLO('yolov8n.pt')

        # Open the webcam (usually index 0)
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("Error: Could not open webcam.")
            return

        prev_time = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Perform object detection
            results = model(frame, verbose=False) #verbose=False silences the image output

            # Get current time for FPS calculation
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time

            # Process and print detection results
            for result in results:
                boxes = result.boxes.cpu().numpy() #move to cpu and convert to numpy
                for box in boxes:
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]
                    confidence = box.conf[0]
                    x1, y1, x2, y2 = map(int, box.xyxy[0]) #convert to int
                    print(f"Class: {class_name}, Confidence: {confidence:.2f}, Box: ({x1}, {y1}, {x2}, {y2}), FPS: {fps:.2f}")

            # Optionally, add a small delay to reduce CPU load (if needed)
            # time.sleep(0.01)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Release the webcam and close all windows
        if 'cap' in locals() and cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_yolo_detection()