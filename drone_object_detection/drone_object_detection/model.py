import os
from ament_index_python.packages import get_package_share_directory
import yaml

from drone_object_detection.yolos import Yolo
import logging

BASE_DIR = get_package_share_directory('drone_object_detection')
with open(os.path.join(BASE_DIR, "conf", "models_conf.yaml"), 'r') as file:
    models_conf = yaml.safe_load(file)


class Model():
    def __init__(self, model_name) -> None:
        self.model_name = model_name 
        self.model_conf = models_conf[self.model_name]
        self.class_names = self.model_conf["class_names"]  
        if len(self.class_names) <= 0:  
            self.class_names = ['Target', 'Square_Circle']
        
        self.model_path = os.path.join(BASE_DIR, self.model_conf["model_weight_path"])
        self.predictor = self.load_model(self.model_name)
        self._type = None # "yolo", "ssd", "fcnn", ...
        self.predict_kwargs = self.model_conf["predict_kwargs"]

        
        
    def load_model(self, model_name):
        if "yolo" in model_name.lower():
            self.type = "yolo"
            logging.info(f'load model from {self.model_path}')
            model = Yolo(self.model_path)
            logging.info(f"model classes : {model.model.names}")
        return model

    def predict(self, image):
        boxes, labels, probs = [], [], []

        if self.type == "yolo":
            boxes, labels, probs = self.predictor.predict(image, **self.predict_kwargs)
            
        
        return boxes, labels, probs 