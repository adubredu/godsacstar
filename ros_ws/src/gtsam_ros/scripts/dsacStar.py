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
import numpy as np

def ssc_to_homo(ssc):

    # Convert 6-DOF ssc coordinate transformation to 4x4 homogeneous matrix
    # transformation

    sr = np.sin(np.pi/180.0 * ssc[3])
    cr = np.cos(np.pi/180.0 * ssc[3])

    sp = np.sin(np.pi/180.0 * ssc[4])
    cp = np.cos(np.pi/180.0 * ssc[4])

    sh = np.sin(np.pi/180.0 * ssc[5])
    ch = np.cos(np.pi/180.0 * ssc[5])

    H = np.zeros((4, 4))

    H[0, 0] = ch*cp
    H[0, 1] = -sh*cr + ch*sp*sr
    H[0, 2] = sh*sr + ch*sp*cr
    H[1, 0] = sh*cp
    H[1, 1] = ch*cr + sh*sp*sr
    H[1, 2] = -ch*sr + sh*sp*cr
    H[2, 0] = -sp
    H[2, 1] = cp*sr
    H[2, 2] = cp*cr

    H[0, 3] = ssc[0]
    H[1, 3] = ssc[1]
    H[2, 3] = ssc[2]

    H[3, 3] = 1

    return H


def cam2rob(H_c):
    # camera calibration matrices
    x_lb3_c = [0.041862, -0.001905, -0.000212, 160.868615, 89.914152, 160.619894]
    x_rob_lb3 = [0.035, 0.002, -1.23, -179.93, -0.23, 0.50]
    H_lb3_c = ssc_to_homo(x_lb3_c)
    H_rob_lb3 = ssc_to_homo(x_rob_lb3)
    H_rob = H_c @ H_lb3_c @ H_rob_lb3
    return H_rob

class dsacStar():
    def __init__(self, weightsDir, focalLength, mode=1, hypotheses=512, threshold=20, inlieralpha=80, maxpixelerror=100):
        self.network = Network(torch.zeros((3)), False)
        self.network.load_state_dict(torch.load(weightsDir))
        self.network = self.network.cuda()
        self.network.eval()
        self.imgTransform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize(int(480)),
            transforms.Grayscale(),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.4], std=[0.25])
        ])
        self.focalLength = focalLength
        self.mode = mode
        self.hypotheses = hypotheses
        self.threshold = threshold
        self.inlieralpha = inlieralpha
        self.maxpixelerror = maxpixelerror
    

    def predict(self, image, opencv=False):
        if not opencv:
            image = io.imread(image)
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
        out_pose = out_pose.numpy()
        H_rob = cam2rob(out_pose)
        return H_rob