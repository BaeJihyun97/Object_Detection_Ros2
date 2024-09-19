model_conf = {
    "yolov8n": {
        "model_weight_path":"/weights/yolo/yolov8n.pt",
        "label_path" : "",
        "class_names" : ['Target', 'Square_Circle']
    },
    "yolov8s": {
        "model_weight_path":"/weights/yolo/yolov8s.pt",
        "label_path" : "",
        "class_names" : ['Target', 'Square_Circle'],
        "predict_kwargs" : {"iou" : 0.5, "conf": 0.6}
    },
    "yolov8m": {
        "model_weight_path":"/weights/yolo/yolov8m.pt",
        "label_path" : "",
        "class_names" : ['Target', 'Square_Circle'],
        "predict_kwargs" : {"iou" : 0.5, "conf": 0.6}
    },
    "yolov8l": {
        "model_weight_path":"/weights/yolo/yolov8l.pt",
        "label_path" : "",
        "class_names" : ['Target', 'Square_Circle'],
        "predict_kwargs" : {"iou" : 0.5, "conf": 0.6}
    }
}
