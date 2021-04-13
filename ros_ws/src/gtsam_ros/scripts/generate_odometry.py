# Generates noisy odometry measurements from SE(3) ground truth poses.
# This will create the files 'odometry_mu.csv' for both the training and test data, as such:
# [SCENE_DIR]/test/odometry_mu.csv
# [SCENE_DIR]/train/odometry_mu.csv

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

# The scene directory is the one containing the 'train/' and 'test/' subdirectories
# scene_dir = '/datadrive/dsacstar/datasets/7scenes_chess'
# scene_dir = '/datadrive/dsacstar/datasets/Cambridge_GreatCourt'
scene_dir = './test_scene'

pos_sigma = .1  # std deviation of zero-mean gaussian noise for position (x,y)
heading_sigma = .1  # std deviation of zero-mean gaussian noise for heading (theta)


# Only returns x, y, theta as 3-tuple
def H_to_coords(H):
    r = R.from_matrix(H[:3, :3])

    eulerAngles = r.as_euler('zyx', degrees=False)
    return H[0, 3], H[1, 3], eulerAngles[2]


def gen_od(pose_path, output_path):
    _, _, files = next(os.walk(pose_path))
    num_images = len(files)
    prefix = files[0][0:-15]  # find the thing before the "000000.pose.txt"

    poses = [['0' for i in range(7)] for j in range(num_images)]
    # Write timestamps
    for i in range(num_images):
        poses[i][0] = f'{i:06}'

    # Iterate over each pose, convert it to x,y,z,phi,psi,theta and add it to poses
    for i in range(0, num_images):
        pose = os.path.join(pose_path, prefix + f"{i:06}.pose.txt")
        H = np.loadtxt(pose, delimiter='\t')
        H = H.astype(float)
        coords = H_to_coords(H)

        # Add zero-mean gaussian noise
        poses[i][1] = str(coords[0] + np.random.normal(0, pos_sigma))
        poses[i][2] = str(coords[1] + np.random.normal(0, pos_sigma))
        poses[i][6] = str(coords[2] + np.random.normal(0, heading_sigma))

    # Write to file
    with open(os.path.join(output_path, 'odometry_mu.csv'), 'w') as f:
        for i in range(num_images):
            for j in range(7):
                f.write(poses[i][j])
                if j != 6:
                    f.write(',')
            f.write('\n')


if __name__ == '__main__':
    if not os.path.isdir(scene_dir):
        print('Please input a valid scene directory path')
    test_pose_path = os.path.join(scene_dir, 'test/poses/')
    train_pose_path = os.path.join(scene_dir, 'train/poses/')
    test_output_path = os.path.join(scene_dir, 'test/')
    train_output_path = os.path.join(scene_dir, 'train/')

    # if not os.path.isdir(destination_dir):
    #     print('Please input a valid output directory path')

    gen_od(test_pose_path, test_output_path)
    gen_od(train_pose_path, train_output_path)

'''
COV:
1326030979526128,0.0004834344591026,1.62086526498792e-07,1.63161699664884e-06,-1.89654097360093e-13,-6.58720775854557e-10,-2.83151316546792e-11,8.39991764243198e-11,5.07516993071641e-10,6.62480457647971e-10,-8.50568312980972e-09,-9.25303722162879e-10,6.44325327172699e-09,-1.53716788479177e-11,1.96018018666806e-07,8.50555293722982e-09,8.48458323508358e-05,-7.1064355892459e-09,-1.64878412050843e-07,8.46854167248993e-05,3.67198082533809e-06,4.58941345593349e-07

MU:
1326030975726043,-0.000420081974830034,-1.86636773560187e-08,-2.48147585340488e-07,-0.000122568168464193,0.000253277828698865,0.000142422056606603
'''
