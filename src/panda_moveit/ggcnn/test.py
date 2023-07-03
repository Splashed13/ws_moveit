from models.ggcnn import GGCNN
import torch

model = GGCNN()
model.load_state_dict(torch.load('/home/tho868/ws_moveit/src/panda_moveit/ggcnn/ggcnn_weights_cornell/ggcnn_epoch_23_cornell_statedict.pt', map_location=torch.device('cpu')))