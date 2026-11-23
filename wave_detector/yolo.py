"""Optional local YOLO pose model adapter; weights are never downloaded here."""
from pathlib import Path


class YoloPoseAdapter:
    def __init__(self,model_path,device='cpu'):
        path=Path(model_path).expanduser()
        if not path.is_file():raise ValueError('Supply an existing local pose-model file')
        from ultralytics import YOLO
        self.model=YOLO(str(path));self.device=device

    def predict(self,image):
        people=[]
        for result in self.model.track(image,persist=True,device=self.device,verbose=False,tracker='bytetrack.yaml'):
            if result.keypoints is None or result.boxes is None or result.boxes.id is None:continue
            for identity,box,points in zip(result.boxes.id.cpu().numpy(),result.boxes.xyxy.cpu().numpy(),result.keypoints.data.cpu().numpy()):
                people.append({'track_id':int(identity),'bbox':box.tolist(),'keypoints':points.tolist()})
        return people
