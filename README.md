# rs_camera pkg

> MSU apple detection project. 
> RealSense camera module.

### Env requirements: 

* Ros: __Kinetic__
* Python: __Python2__
* Network: MSU EGR Network

### Prerequisites: 

1️⃣ Packages

```shell
pip2 install pyrealsense2
```

2️⃣ Environment Setup

```shell
# Ensure use the same master in the LAN.
echo 'export ROS_MASTER_URI=http:35.9.138.211//:11311/' >> ~/.bashrc 
echo 'export ROS_IP=`hostname -I`' >> ~/.bashrc 
# if you use zsh, please change .bashrc to .zshrc
```

### Install & Run

```shell
# enter your ros workspace
cd ~/catkin_ws/src

git clone https://github.com/bennie-msu/rs_camera.git
cd ~/catkin_ws
catkin_make 
# You should setup it when you open any new session or you can add it into .bashrc
source ~/catkin_ws/devel/setup.bash

# Run
rosrun rs_camera main.py
```

Now It's done 😊😊😊


### Protocol

| Topic       | Data Type                                                    | Description                                                  |
| ----------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| color_image | [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | Color images from RS camera                                  |
| depth_image | [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | gray-scale depth image from RS camera                        |
| detections  | [Int32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | Bounding Boxes of apples detection in the form of (x1, y1, x2, y2) |
| positions   | [PoseArray](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseArray.html) | A sequence position data for each apple, organized in (x, y, z) |
