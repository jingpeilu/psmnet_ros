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
If have different camera parameters, change this line on config file.
```
camera_file: ./camera_calibration.yaml
```
Subscriber topics:
```
left_image_sub: /stereo/slave/left/image #(1920,1080)
right_image_sub: /stereo/slave/right/image #(1920,1080)
mask_image_sub: /stereo/viewer/left/image #(1920,1080)
```
Publisher topics:
```
depth_publisher: camera/depth_image #(640,480)
color_publisher: camera/color_image #(640,480)
```

If has libcv_bridge.so error, specify the LD_LIBRARY_PATH to your anaconda environment:
```
export LD_LIBRARY_PATH=/home/$USER/anaconda3/envs/py27/lib:/opt/ros/melodic/lib
```
