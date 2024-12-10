import cv2
import h5py

data_file = r"/home/johnathant/VIOLA-master/datasets/StackTwoTypesDomain_training_set/demo.hdf5"

f = h5py.File(data_file, "r")
print(f["data"]["demo_0"]["obs"]['agentview_rgb'])
img = f["data"]["demo_0"]["obs"]['agentview_rgb'][0]

img2 = f["data"]["demo_0"]["obs"]['eye_in_hand_rgb'][0]

cv2.imshow("temp_img", img)
cv2.imshow("temp_img2", img2)
cv2.waitKey(0)
