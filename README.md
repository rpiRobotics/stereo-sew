# Stereographic SEW Angle

Implementation of stereographic SEW (shoulder-elbow-wrist) angle for 7-DOF robot arms as well as inverse kinematics solutions for a number of 7-dof arms from ["Redundancy parameterization and inverse kinematics of 7-DOF revolute manipulators"](https://arxiv.org/abs/2307.13122).
The IK procedures also work with the conventional SEW angle.

For 6-dof IK solutions see [linear-subproblem-solutions](https://github.com/rpiRobotics/linear-subproblem-solutions). Code in this repo depends on that code.

Also depends on [matlab-diagrams](https://github.com/aelias36/matlab-diagrams) for a few visualizations

## Folder Breakdown

`+SEW_IK`: Inverse kinematics solutions for different robot types

`+SEW_IK_setups`: Test classes for IK solutions

`+robot_kin`: Kinematic parameters for robot examples

`IK_helpers`: Functions to help with robot IK (and forward kinematics)

`SEW_comparison_demo`: Demonstration of stereographic vs conventional SEW angle

`correctness_tests`: Tests to verify the correctness IK solutions

`gui`: Interactive demo for inverse kinematics solutions (work in progress)

`robot_examples`: Demonstration of IK for specific robots

`ros`: Code to communicate with Sawyer robot over ROS

`unit_tests`: Testing for smaller helper functions

`verify_eqns`: Verify equations used in paper


## Contributing
If you have any improvements you'd like to make, or even ideas or requests for improvements, please start a GitHub issue.
