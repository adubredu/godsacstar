import rospy
import rosbag
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
# from sensor_msgs.msg import header
from cv_bridge import CvBridge
import os


bridge = CvBridge()

IMAGE_FOLDER = '/home/jingyu/resource/nclt_image/'
OUTPUT_BAG = 'nclt_lb3.bag'

output_bag = rosbag.Bag(OUTPUT_BAG, 'w')
img_seq = os.listdir(IMAGE_FOLDER)
# print(img_seq)
for img in img_seq:
    bag_img = cv2.imread(IMAGE_FOLDER + img)
    
    bag_img = bridge.cv2_to_imgmsg(bag_img)
    img_header = Header()
    stamp = rospy.Time(int(img[11:21]), int(img[21:-10]))
    img_header.stamp = stamp
    bag_img.header = img_header
    output_bag.write('nclt/lb3', bag_img, t=stamp)

output_bag.close()
    # int(image[11:-10])