This is Chuan's final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Due to limited time, I am late for finishing the capstone project on time with other team members, so I am submitting this project alone.

As mentioned in the final project walkthroughs, I finished this project in 3 parts:

## Waypoint Updater

Waypoint updater node contains two parts. The first part is to derive a fixed number (30) of waypoints ahead of car from overall waypoints without consideration of traffic lights. Firstly, I use two callback functions "pose_cb" and "waypoints_cb" to get current car position and overall waypoints from two topics "/current_pose" and "/base_waypoints". Then a function called **get_closest_waypoint_idx** is created to get the closest waypoint ahead of car. At last, from that closest waypoint, I add additional 30 points from it to generate the final waypoints for car to follow. 

The second part of waypoint updater is to incorporate traffic light stopping. First, from topic "/traffic_waypoints" I get the stop line waypoint. Based this stop line waypoint, I designed a function called "decelerate_waypoints" to slow down the car to stop at red light. This is mainly done through modifying the velocity of those final waypoints.

## DBW

Driver by wire node (dbw_node.py) contains mainly a twist controller (twist_controller.py) to control vehicle speed to follow target vehicle speed from final waypoints via a PID controller, and also to control vehicle steering angle to follow target vehicle yaw rate/yaw angle/lateral position via a combined feedforward steering controller and feedback PID steering controller. The feedback steering PID controller is using cross track error as input as shown in (cte_calculator.py).

## Traffic light detection

Traffic light detection node is used to generate the traffic light waypoints which are sent to waypoints updater node. First, I need to get all the necessary inputs from subscribing necessary topics such as "/current_pose", "/base_waypoints", "/image_color" to know the information about current vehicle position, overall track waypoints, and the traffic light image data from camera. After that, these information are used to calculate the closest points to the red traffic light. This is realized in a designed function of process_traffic_lights. This function calculates the waypoints closest to the upcoming stop line position, and also use the traffic light classifier to classify the detected camera image as one of these states: 0-RED, 1-YELLOW, 2-GREEN, 4:UNKOWN. The returned values of stop line waypoint combined with traffic light state are used to decide final stop line waypoint with some hysterisis characterics to make the dectection more stable.

The traffic light classifier is based on a CNN model for light detection on the simulator. 

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
