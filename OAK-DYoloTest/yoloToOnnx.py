from ultralytics import YOLO

model_name = 'yolov8n' #@param ["yolov8n", "yolov8s", "yolov8m", "yolov8l", "yolov8x"]
input_width = 640 #@param {type:"slider", min:32, max:4096, step:32}
input_height = 640 # @param {"type":"slider","min":32,"max":4096,"step":32}
optimize_cpu = False

model = YOLO(f"{model_name}.pt") 
model.export(format="onnx", imgsz=[input_height,input_width], optimize=optimize_cpu)