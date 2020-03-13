# psmnet_ros
Stereo reconstruction using PSMNet (SuPer Deep implementation)

## Usage

### Dependencies
Recommend set up the environment using Anaconda ("environment.yml" provides an example enviornment).

- Python2.7
- PyTorch(0.4.0+)
- torchvision 0.2.0 
- rospkg 1.1.10

Install rospkg in conda environment:
```
$ conda install setuptools
$ pip install -U rosdep rosinstall_generator wstool rosinstall six vcstools
```

### Run File
To run the file:
```
python PSMNet_ros.py
```

### Notes
If have different camera parameters, change this line to your calibration file.
```
camera_file = "./camera_calibration.yaml"
```
Subscriber topics:
```
image_sub_left = Subscriber("/stereo/slave/left/image", Image)  #(1920,1080)
image_sub_right = Subscriber("/stereo/slave/right/image", Image)  #(1920,1080)
image_sub_mask = Subscriber("/stereo/viewer/left/image", Image) #(1920,1080)
```
Publisher topics:
```
depth_pub = rospy.Publisher("camera/depth_image",Image,queue_size=10) #(640,480)
image_pub = rospy.Publisher("camera/color_image",Image,queue_size=10) #(640,480)
```

If has libcv_bridge.so error, specify the LD_LIBRARY_PATH to your anaconda environment:
```
export LD_LIBRARY_PATH=/home/$USER/anaconda3/envs/py27/lib:/opt/ros/melodic/lib
```
