# psmnet_ros
Stereo reconstruction using PSMNet (SuPer Deep implementation)

## Usage

### Dependencies
Recommend set up the enviornment using Anaconda ("enviornment.yml" provides an example enviornment).

- Python2.7
- PyTorch(0.4.0+)
- torchvision 0.2.0 
- rospkg 1.1.10

Install rospkg in conda enviornment:
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
"/stereo/slave/left/image"  Image(1920,1080)
"/stereo/slave/right/image" Image(1920,1080)
"/stereo/viewer/left/image" Image(1920,1080)

Publisher topics:
"camera/depth_image"  Image(640,480)
"camera/color_image"  Image(640,480)

