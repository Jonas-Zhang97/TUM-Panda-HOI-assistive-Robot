# Settings for MoveIt HandEye Calibration
## Target
### Target Params
For settings in Target Params, please see setting_target.png.
### Target Pose Detection
Image Topic: /camera/color/image_raw

CameraInfo Topic: /camera/color/camera_info
## Context
### General Setting
Sensor configuration: Eye-to-hand
### Frames Selection
Sensor frame: camera_link

Object frame: handeye_target

End effector frame: panda_hand_tcp

Robot base frame: panda_link_0
### Camera Initial Guess
Let all the settings in Camera Pose Initial Guess stay as default