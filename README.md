# Path Planning for Lynx Robots on ROS/Gazebo
This is a path planning for Lynx robot based on Gazebo and ROS

## Proposal

The goal and our method : [Proposal](proposal.pdf)
We would like to achive some planning method such as RRT, RRT* and SST to our simulator


## Part One: The Gazebo Model

We refer to the model in this repo https://github.com/gdepaepe/al5d_description    
It's the initial model without the exact shape of Lynxmotion arm.


### Usage

#### 0. Install ROS, Gazebo and Moveit!

#### 1. set up the ros workspace

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


#### 2. Launch Gazebo Model

```
roslaunch lynx_gazebo lynx_gazebo.launch
```

## Part Two: Manipulation with MoveIt!

run and try planning on MoveIt
```
roslaunch lynx_moveit demo.launch
```





## Part Three: 
## Reference


