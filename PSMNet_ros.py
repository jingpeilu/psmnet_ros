from __future__ import print_function
import os
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data
from torch.autograd import Variable
import torch.nn.functional as F
import skimage
import skimage.io
import skimage.transform
from skimage import img_as_ubyte
import numpy as np
import time
from utils import preprocess 
from models import *
import cv2
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

rospy.init_node('psmnet')
bridge = CvBridge()
camera_file = "./camera_calibration.yaml"

############### read camera matrix ################
fs = cv2.FileStorage(camera_file, cv2.FILE_STORAGE_READ)
fn = fs.getNode("K1")
mtx1 = fn.mat()
mtx1 = mtx1/2
mtx1[2,2] = 1
fn = fs.getNode("K2")
mtx2 = fn.mat()
mtx2 = mtx2/2
mtx2[2,2] = 1
fn = fs.getNode("D1")
dist1 = fn.mat()
fn = fs.getNode("D2")
dist2 = fn.mat()
fn = fs.getNode("R")
R = fn.mat()
fn = fs.getNode("T")
T = fn.mat()

BINIMG_W = 640
BINIMG_H = 480

R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    mtx1, dist1,
    mtx2, dist2,
    (960, 540),
    R,
    T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    alpha=0,
    newImageSize=(BINIMG_W, BINIMG_H)
)
def rectify_undistort(img1,img2):
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        mtx1, dist1,
        mtx2, dist2,
        (960, 540),
        R,
        T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,
        newImageSize=(BINIMG_W, BINIMG_H)
    )
    map11,map12 = cv2.initUndistortRectifyMap(mtx1,dist1,R1,P1,(BINIMG_W, BINIMG_H),cv2.CV_16SC2)
    map21,map22 = cv2.initUndistortRectifyMap(mtx2,dist2,R2,P2,(BINIMG_W, BINIMG_H),cv2.CV_16SC2)
    img1r = cv2.remap(img1, map11, map12, cv2.INTER_LINEAR)
    img2r = cv2.remap(img2, map21, map22, cv2.INTER_LINEAR)
    return img1r,img2r


def apply_mask(disp,image_mask):
    img_mask = cv2.resize(image_mask, (854,480), interpolation = cv2.INTER_AREA)
    img_mask = img_mask[:,:,2]
    img_mask[img_mask<254] = 0
    img_mask[img_mask>=254] = 1
    kernel = np.ones((9,9), np.uint8) 
    img_mask = cv2.erode(img_mask, kernel, iterations=1) 
    offset = 100
    img_mask = img_mask[:,offset:offset+640]
    return disp * img_mask
    
    
################### load PSMNet ######################

torch.manual_seed(6)
torch.cuda.manual_seed(6)

model = stackhourglass(192)
#model = basic(192)


model = nn.DataParallel(model, device_ids=[0])
model.cuda()


print('load PSMNet')
state_dict = torch.load("pretrained_model_KITTI2015.tar")
model.load_state_dict(state_dict['state_dict'])

print('Number of model parameters: {}'.format(sum([p.data.nelement() for p in model.parameters()])))

def test(imgL,imgR):
    model.eval()

    imgL = torch.FloatTensor(imgL).cuda()
    imgR = torch.FloatTensor(imgR).cuda()     

    imgL, imgR= Variable(imgL), Variable(imgR)

    with torch.no_grad():
        disp = model(imgL,imgR)

    disp = torch.squeeze(disp)
    pred_disp = disp.data.cpu().numpy()

    return pred_disp

processed = preprocess.get_transform(augment=False)

def cal_disp(imgL_o,imgR_o):

    imgL = processed(imgL_o).numpy()
    imgR = processed(imgR_o).numpy()
    imgL = np.reshape(imgL,[1,3,imgL.shape[1],imgL.shape[2]])
    imgR = np.reshape(imgR,[1,3,imgR.shape[1],imgR.shape[2]])

    # pad to width and hight to 16 times
    if imgL.shape[2] % 16 != 0:
        times = imgL.shape[2]//16       
        top_pad = (times+1)*16 -imgL.shape[2]
    else:
        top_pad = 0
    if imgL.shape[3] % 16 != 0:
        times = imgL.shape[3]//16                       
        left_pad = (times+1)*16-imgL.shape[3]
    else:
        left_pad = 0     

    imgL = np.lib.pad(imgL,((0,0),(0,0),(top_pad,0),(0,left_pad)),mode='constant',constant_values=0)
    imgR = np.lib.pad(imgR,((0,0),(0,0),(top_pad,0),(0,left_pad)),mode='constant',constant_values=0)

    start_time = time.time()
    pred_disp = test(imgL,imgR)
    print('prediction time = %.2f' %(time.time() - start_time))
    if top_pad !=0 or left_pad != 0:
        img = pred_disp[top_pad:,:-left_pad]
    else:
        img = pred_disp
        
    return img

#publisher
depth_pub = rospy.Publisher("camera/depth_image",Image,queue_size=10)
image_pub = rospy.Publisher("camera/color_image",Image,queue_size=10)


def gotimage(image_l, image_r, image_m):
    #bridge = CvBridge()
    l_stamp = image_l.header.stamp
    print('receiving images: ' + str(l_stamp))
    try:
        image_left = bridge.imgmsg_to_cv2(image_l, desired_encoding="passthrough")
        image_right = bridge.imgmsg_to_cv2(image_r, desired_encoding="passthrough")
        image_mask = bridge.imgmsg_to_cv2(image_m, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)
    #assert image_left.header.stamp == image_right.header.stamp
    img_l = cv2.resize(image_left, (960,540), interpolation = cv2.INTER_AREA)
    img_r = cv2.resize(image_right, (960,540), interpolation = cv2.INTER_AREA)
    img_l,img_r = rectify_undistort(img_l,img_r)
    disp = cal_disp(img_l,img_r)

    # convert to depth map

    pcl = cv2.reprojectImageTo3D(disp,Q)
    depth = pcl[:,:,2]*10

    # apply mask
    depth = apply_mask(depth,image_mask)
    depth[0:150,:] = 0
    depth = depth.astype('uint16')
    #depth[depth<600] = 0
    #depth[depth<716] = 0
    print('publishing depth map: ' + str(l_stamp))
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(img_l,encoding='rgb8'))
        depth_pub.publish(bridge.cv2_to_imgmsg(depth,encoding='mono16'))
    except CvBridgeError as e:
        print(e)
    #time.sleep(0.5)

image_sub_left = Subscriber("/stereo/slave/left/image", Image)
image_sub_right = Subscriber("/stereo/slave/right/image", Image)
image_sub_mask = Subscriber("/stereo/viewer/left/image", Image)

ats = ApproximateTimeSynchronizer([image_sub_left, image_sub_right, image_sub_mask], queue_size=100, slop=0.5)
ats.registerCallback(gotimage)


# run loop
while not rospy.is_shutdown():

    try:
        rospy.spin()
    except:
        print("error")
