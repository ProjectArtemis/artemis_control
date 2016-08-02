# Artemis Control

This package contains several ROS nodes for trajectory generation and fast closed-loop control of UAVs.

## SE(3) Trajectory Controller 

```trajectory_controller_node``` is SE(3) trajectory controller for multirotors based on the paper [Geometric Tracking Control of a Quadrotor UAV on SE(3)](http://www.math.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf). The nonlinear tracking controller uses a rigid body dynamics model of multirotors and is developed on the special Euclidean group SE(3). The controller directly outputs rotor velocity commands.

The code is based on ETH ASL's [RotorS simulator](https://github.com/ethz-asl/rotors_simulator/) package.

Instructions for system identification TODO.

## Kinodynamic Trajectory Generator

TODO
