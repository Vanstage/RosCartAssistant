# WID3010-G13
Robotics development to assist the elderly.

## RosCartAssistant

RosCartAssistant is a project that utilizes ROS (Robot Operating System) for a shopping cart assistant. This README provides instructions on how to set up and run the project.



## Requirements

This project was developed in Jupiter IO - Juno robot with preconfig and preinstalled OpenVINO Yolo model. To run this project, you need the following software installed on your Linux system (Ubuntu 18.04 and ROS Melodic):

- ROS (Robot Operating System): Ubuntu 18.04
- OpenVINO: For running the YOLO model. Follow the installation guide.
- USB Camera Driver: Ensure the usb_cam package is installed. 
- Python Dependencies: Install required Python packages. 
- xterm: Ensure xterm is installed for launching nodes in new terminal windows.

You can install the required ROS packages and dependencies using the following commands:

```sh
sudo apt update
sudo apt install ros-melodic-usb-cam
sudo apt install xterm
pip install -r requirements.txt # Make sure you have a requirements.txt with the necessary Python packages
```

If you are using the Jupiter Juno  robot, just install the  Python requirements using the following command:

```sh
pip install -r requirements.txt # Make sure you have a requirements.txt with the necessary Python packages
```

Installation
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

Running the Launch File
To run the project, launch the provided launch file. This will start all the necessary nodes for the shopping cart assistant.

```sh
roslaunch shopping_cart_assistant ros_cart_assistant.launch
```
