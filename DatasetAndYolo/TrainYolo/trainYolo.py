from ultralytics import YOLO

def main():
    # Load a pretrained YOLO11n model
    model = YOLO("best.pt")
    # Na mojem računalniku porabi 7GB VRAM-a, in 25GB RAM-a
    train_results = model.train(
        data="dataset1/data.yaml",  # Path to dataset configuration file
        epochs=100,     # Number of epochs to train
        imgsz=640,      # Image size for training
        batch=45,       # Batch size for training
        workers=16,     # Number of workers for data loading
        device=0,       # Device to use for training (0 for GPU, 'cpu' for CPU)
        amp=True,
    )

    # Evaluate the model's performance on the validation set
    metrics = model.val()

    # Perform object detection on an image
    results = model("test.jpg")  # Predict on an image
    results[0].show()  # Display results
    results = model("test1.jpg")  # Predict on an image
    results[0].show()  # Display resultsresults = model("test.jpg")  # Predict on an image
    results = model("test2.jpg")  # Predict on an image
    results[0].show()  # Display resultsresults = model("test.jpg")  # Predict on an image
    # Export the model to ONNX format for deployment
    #path = model.export(format="openvino")  # Returns the path to the exported model
    #print(f"Model exported to {path}")

if __name__ == '__main__':
    main()