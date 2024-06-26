# Author:  Mustafa Keskin(s224955).

#Importing libraries
from ultralytics import YOLO

# Load a final_v17 PyTorch model
model = YOLO("final_v17.pt")

# Export the model
model.export(format="openvino")  # creates 'final_v17_openvino_model/'

# Load the exported OpenVINO model
ov_model = YOLO("final_v17_openvino_model/")
