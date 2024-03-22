# Stereographic SEW Angle

Implementation of stereographic SEW (shoulder-elbow-wrist) angle for 7-DOF robot arms as well as inverse kinematics solutions for a number of 7-dof arms from ["Redundancy parameterization and inverse kinematics of 7-DOF revolute manipulators"](https://arxiv.org/abs/2307.13122).
The IK procedures also work with the conventional SEW angle.

Abstract: Seven degree-of-freedom (DOF) robot arms have one redundant DOF which does not change the motion of the end effector. The redundant DOF offers greater manipulability of the arm configuration to avoid obstacles and singularities, but it must be parameterized to fully specify the joint angles for a given end effector pose. For 7-DOF revolute (7R) manipulators, we introduce a new concept of generalized shoulder-elbow-wrist (SEW) angle, a generalization of the conventional SEW angle but with an arbitrary choice of the reference direction function. The SEW angle is widely used and easy for human operators to visualize as a rotation of the elbow about the shoulder-wrist line. Since other redundancy parameterizations including the conventional SEW angle encounter an algorithmic singularity along a line in the workspace, we introduce a special choice of the reference direction function called the stereographic SEW angle which has a singularity only along a half-line, which can be placed out of reach. We prove that such a singularity is unavoidable for any parameterization. We also include expressions for the SEW angle Jacobian along with singularity analysis. Finally, we provide efficient and singularity-robust inverse kinematics solutions for most known 7R manipulators using the general SEW angle and the subproblem decomposition method. These solutions are often closed-form but may sometimes involve a 1D or 2D search in the general case. Search-based solutions may be converted to finding zeros of a high-order polynomial. Inverse kinematics solutions, examples, and evaluations are available in a publicly accessible repository.

For examples using the polynomial method, see the [subproblem-polynomial](https://github.com/rpiRobotics/subproblem-polynomial) repo.

For 6-DOF IK solutions see [ik-geo](https://github.com/rpiRobotics/ik-geo). Code in this repo depends on that code.

This repo also depends on [matlab-diagrams](https://github.com/aelias36/matlab-diagrams) for a few visualizations.

## Interactive Demos

[Conventional SEW Angle](https://www.geogebra.org/m/ftpsw5ut)

[Stereographic SEW Angle](https://www.geogebra.org/m/z4ss2jmg)

## Multimedia Extensions

[Comparing conventional and stereographic SEW angles](https://www.youtube.com/watch?v=Gc-zbK4IfPU)

[Sawyer inverse kinematics solutions using 1D search](https://www.youtube.com/watch?v=4MpwNNHUA58)


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

If you have any questions, improvements you'd like to make, or even ideas or requests for improvements, please start a GitHub issue or send an email.
