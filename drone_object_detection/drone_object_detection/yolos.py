'''
TODO
모델 경량화를 위해 변경될 예정입니다.
'''
from ultralytics import YOLO
import torch


class Yolo():
    def __init__(self, model_path):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = YOLO(model_path)
        self.model.to(self.device)
        
    def predict(self, image, **kwargs):
        boxes, labels, probs = [], [], []
        
        results =  self.model.predict(image, save=False, verbose=False, **kwargs) # , verbose=False
        data = results[0].cpu().boxes.data
        for box in data:
            x1, y1, x2, y2, prob, label = box.tolist()
            boxes.append([x1, y1, x2, y2])
            probs.append(prob)
            labels.append(int(label))

        return boxes, labels, probs

