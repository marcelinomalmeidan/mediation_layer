# mediation_layer

This package handles collision avoidance for multiple vehicles in an enclosed volume. This repository makes sure that no vehicle
exits the volume, neither allow them to collide among themselves. 

The Mediation Layer package is based on potential fields, repelling vehicles that are too close to each other, as well as repelling
vehicles from the boundary walls. When no repulsion forces are acting on a quadcopter, the Mediation Layer's output converges to the 
desired reference. This package can be summarized in the following image:

![alt text](https://github.com/marcelinomalmeidan/mediation_layer.git/master/path/to/img.png)

In the image above, y_{ref} is a desired reference, and y*_{ref} is the reference modified by the Mediation Layer. Note that the mediation layer uses the current state of the system X (position of the quadcopters) to compute y*_{ref}.

## Launch file parameters

This package is commonly executed using parameters from a .launch file. The launch file sets parameters such as:

- Quadcopter names: this is crucial for the mediation_layer to know which topics to listen to and to publish into. 

- TODO

## Inputs and Outputs

When running, the mediatio_layer will be subscribing to two types of topics:

- Desired references for the quadcopters

- Current state of the quadcopters

## Dependencies

- This software was developed for ROS Kinetic in Ubuntu 16.04. We haven't tested for other distributions. See installation procedure in http://wiki.ros.org/kinetic/Installation/Ubuntu.

- Eigen

```sudo apt-get install libeigen3-dev ```

- MAVROS

``` sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras```

- It also depends on the px4_control package, which is the package we use to perform position control with any px4-based quadcopter. The mediation_layer package uses a message definition that is defined in px4_control.

```
cd ~/catkin_ws/src
git clone https://github.com/radionavlab/px4_control.git
cd ..
catkin_make
```

## Compiling

- Copy the present REPO into a catkin workspace, e.g.:

```
cd ~/catkin_ws/src
git clone https://github.com/marcelinomalmeidan/px4_control.git
```

- Compile the catkin workspace, e.g.:

```
cd ~/catkin_ws
catkin_make
```## Compiling

- Copy the present REPO into a catkin workspace, e.g.:

```
cd ~/catkin_ws/src
git clone https://github.com/marcelinomalmeidan/mediation_layer.git
```

- Compile the catkin workspace, e.g.:

```
cd ~/catkin_ws
catkin_make
```

