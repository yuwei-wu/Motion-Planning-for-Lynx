# Path Planning for Lynx Robots on ROS/Gazebo
This is a path planning for Lynx robot based on Gazebo and ROS

<p align="center">
   <img src="docs/img/2.gif">
</p>


## Proposal

The goal and our method : [Proposal](docs/proposal.pdf)
We would like to achive some planning method such as RRT, RRT* and SST to our simulator


## Part One: Gazebo Model

We refer to the urdf file in this repo https://github.com/gdepaepe/al5d_description


### 1. Parameters of the robot

we have check the robot parameters of the model with 5l5d

which is in lynx_descripton/lynx.xacro
```
    <xacro:property name="base_height" value="0.043" />
    <xacro:property name="base_radius" value="0.048" />
    <xacro:property name="upper_base_height" value="0.006" />
    <xacro:property name="upper_arm_offset" value="0.022" />
    <xacro:property name="upper_arm_length" value="0.14605" />
    <xacro:property name="upper_arm_width" value="0.05" />
    <xacro:property name="lower_arm_length" value="0.187325" />
    <xacro:property name="lower_arm_width" value="0.035" />
    <xacro:property name="wrist_length" value="0.055" />
    <xacro:property name="wrist_width" value="0.025" />
    <xacro:property name="gripper_length" value="0.06" />
    <xacro:property name="gripper_width" value="0.05" />
    <xacro:property name="gripper_height" value="0.02" />
    <xacro:property name="finger_length" value="0.04" />
```


### 2.Usage

#### (1) Install ROS, Gazebo and Moveit!

#### (2) Set up the ros workspace

ROS tutorial here: [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

```
mkdir lynx_ws/src
cd mkdir lynx_ws/src
git clone https://github.com/yuwei-wu/Motion-Planning-for-Lynx.git
catkin_make
```

remember to source or add path in .bashrc file

```
source devel/setup.bash
source /opt/ros/kinetic/setup.bash
```


#### (3) Launch Gazebo Model

```
roslaunch lynx_gazebo lynx_gazebo.launch
```



<p align="center">
   <img src="docs/img/2.gif">
</p>






## Part Two: Manipulation with MoveIt!

run and try planning on MoveIt
```
roslaunch lynx_moveit demo.launch
```





## Part Three: Our Motion Planning

 
## Reference




