# WID3010-G13
Robotics development to assist the elderly.

## RosCartAssistant

RosCartAssistant is a project that utilizes ROS (Robot Operating System) for a shopping cart assistant prototype by utilise speech recognition and object detection. This README provides instructions on how to set up and run the project.


## Requirements

This project was developed in Jupiter IO - Juno robot with preconfig and preinstalled OpenVINO Yolo model. To run this project, you need the following software installed on your Linux system (Ubuntu 18.04 and ROS Melodic):

- **ROS (Robot Operating System)**: Ros Melodic Ubuntu 18.04
- **OpenVINO**: For running the YOLO model.
- **USB Camera Driver**: Ensure the usb_cam package is installed. 
- **Python Dependencies**: Install required Python packages. 
- **xterm**: Ensure xterm is installed for launching nodes in new terminal windows.
- **mpg321**: Play the generated MP3 file for gTTS (Google Text-to-Speech)


If you are using the Jupiter Juno robot, you can install the required ROS packages and dependencies using the following commands:

```sh
sudo apt update
sudo apt install xterm
sudo apt install mpg321
pip install -r requirements.txt # Make sure you have a requirements.txt with the necessary Python packages
```

##Prototype Information
This cart assistant uses YOLOv3 from OpenVINO for object detection. The object detection results are used to interact with the cart system. Currently, the objects detected are general as it use pre-trained model existed in Jupiter Juno robot.

##Installation
Clone the repository:
```sh
git clone https://github.com/Vanstage/RosCartAssistant.git
cd RosCartAssistant
```

Build the workspace:
```sh
catkin_make
```

Source the setup file:
```sh
source devel/setup.bash
```

##Running the Launch File

To run the project, launch the provided launch file. This will start all the necessary nodes for the shopping cart assistant.

```sh
roslaunch shopping_cart_assistant ros_cart_assistant.launch
```

##Available Commands

-**Price**: Inquire about the prices of detected objects.-
-**Repeat**: Repeat the last spoken text.-
-**Add to Cart**: Add detected objects to the cart.-
-**List Item**: List items currently in the cart.-
-**Total**: Calculate the total price of items in the cart.-
-**Done**: Finish cart assistant.-

##Launch File
The launch file is used to start multiple nodes required for the cart assistant, including the USB camera node, YOLOv3 object detection node, image viewer node, speech recognition node, and text-to-speech node.

Launch File: 'cart_assistant.launch':
```sh
<?xml version="1.0" encoding="utf-8"?>
<launch>
  <!-- Console launch prefix -->
  <arg name="launch_prefix" default=""/>
  
  <!-- Model folder -->
  <arg name="yolo_model_folder" default="$(find robot_vision_openvino)/models/yolov3"/>

  <!-- Load ROS parameters -->
  <!-- <rosparam file="$(find robot_vision_openvino)/config/ros.yaml" command="load" ns="yolo_ros" /> -->
  <rosparam file="$(find robot_vision_openvino)/config/yolov3.yaml" command="load" ns="yolo_ros" />

  <!-- Start USB camera node -->
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" respawn="true">
    <param name="video_device" value="/dev/video2" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="camera_frame_id" value="usb_cam" />
  </node>

  <!-- Start OpenVINO YOLO node in a new xterm terminal -->
  <node name="yolo_ros" pkg="robot_vision_openvino" type="yolo_ros" output="screen" launch-prefix="xterm -e" args="$(arg launch_prefix)">
    <param name="yolo_model/folder" value="$(arg yolo_model_folder)" />
  </node>

  <!-- Start image viewer node in a new xterm terminal -->
  <node name="image_viewer" pkg="shopping_cart_assistant" type="image_viewer.py" output="screen" launch-prefix="xterm -e" args="$(arg launch_prefix)" />

  
  <!-- Start speech recognition node in a new xterm terminal -->
  <node name="cart_assistant_sr_node" pkg="shopping_cart_assistant" type="cart_assistant_sr.py" output="screen" launch-prefix="xterm -e" args="$(arg launch_prefix)" />

  <!-- Start text-to-speech (TTS) node in a new xterm terminal -->
  <node name="cart_assistant_tts_node" pkg="shopping_cart_assistant" type="cart_assistant_tts.py" output="screen" launch-prefix="xterm -e" args="$(arg launch_prefix)" />
</launch>
```

