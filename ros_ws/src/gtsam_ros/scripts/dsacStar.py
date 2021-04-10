# dsac imports
import sys
sys.path.append('/home/tannerliu/Software/posenet_gtsam/dsacstar')
import torch
import cv2
import dsacstar
from network import Network
from skimage import io
from torchvision import transforms
from matplotlib import pyplot as plt


class dsacStar():
    def __init__(self, weightsDir, focalLength, mode=1, hypotheses=64, threshold=10, inlieralpha=100, maxpixelerror=100):
        self.network = Network(torch.zeros((3)), False)
        self.network.load_state_dict(torch.load(weightsDir))
        self.network = self.network.cuda()
        self.network.eval()
        self.imgTransform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize(int(480)),
            transforms.Grayscale(),
            transforms.ColorJitter(brightness=0.1, contrast=0.1),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.4], std=[0.25])
        ])
        self.focalLength = focalLength
        self.mode = mode
        self.hypotheses = hypotheses
        self.threshold = threshold
        self.inlieralpha = inlieralpha
        self.maxpixelerror = maxpixelerror
    
    def predict(self, imageDir):
        image = io.imread(imageDir)
        image = self.imgTransform(image)
        image = image.unsqueeze(0)
        image = image.cuda()
        scene_coordinates = self.network(image)
        scene_coordinates = scene_coordinates.cpu()
        out_pose = torch.zeros((4, 4))
        dsacstar.forward_rgb(
            scene_coordinates, 
            out_pose, 
            self.hypotheses, 
            self.threshold,
            self.focalLength, 
            float(image.size(3) / 2), #principal point assumed in image center
            float(image.size(2) / 2), 
            self.inlieralpha,
            self.maxpixelerror,
            self.network.OUTPUT_SUBSAMPLE)
        out_pose = out_pose.inverse().numpy()
        return out_pose