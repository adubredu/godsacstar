import numpy as np
import matplotlib.pyplot as plt
 
sample = np.loadtxt('/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/helper/2012-01-08_sample.txt')
# sample = sample[0:500,:]
plt.scatter(sample[:,2], sample[:,1], label='*')

gtsam_x = np.load("/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/helper/gtsam_x.npy")
gtsam_y = np.load("/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/helper/gtsam_y.npy")

for i in range(len(gtsam_x)):

    real_x = sample[i, 2]
    real_y = sample[i, 1]
    est_x = gtsam_x[i]
    est_y = gtsam_y[i]
    print(str(sample[i, 0]) + " real_x:" + str(real_x) + " est_x: " + str(est_x) + " real_y:" + str(real_y) + " est_y: " + str(est_y))

plt.scatter(gtsam_y, gtsam_x)
plt.show()