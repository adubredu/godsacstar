from operator import gt
import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
 
sample = np.loadtxt('2012-01-08_sample.txt')
# sample = sample[0:500,:]
# plt.scatter(sample[:,2], sample[:,1], label='*')

gtsam_x = np.load("gtsam_x.npy")
gtsam_y = np.load("gtsam_y.npy")
# gtsam_x = np.load("predict_x.npy")
# gtsam_y = np.load("predict_y.npy")

gt_x = []
gt_y = []
pred_x = []
pred_y = []
for i in np.arange(0, 1500, 1):

    real_x = sample[i, 1]
    real_y = sample[i, 2]
    est_x = gtsam_x[i]
    est_y = gtsam_y[i]
    gt_x.append(real_x)
    gt_y.append(real_y)
    pred_x.append(est_x)
    pred_y.append(est_y)
    print(str(sample[i, 0]) + " real_x:" + str(real_x) + " est_x: " + str(est_x) + " real_y:" + str(real_y) + " est_y: " + str(est_y))

plt.scatter(gt_x, gt_y, label='GT')

plt.scatter(pred_x, pred_y, label='Pred')

# for i in range(len(gt_x)):
#     line_x = []
#     line_y = []
#     line_x.append(gt_x[i])
#     line_x.append(pred_x[i])
#     line_y.append(gt_y[i])
#     line_y.append(pred_y[i])
#     plt.plot(line_x, line_y)

plt.show()
print(len(gt_x))
print(len(gt_y))
gt_x = np.array(gt_x)
gt_y = np.array(gt_y)
pred_x = np.array(pred_x)
pred_y = np.array(pred_y)
error = np.sqrt((gt_x-pred_x)**2 + (gt_y-pred_y)**2)
print(np.mean(error))