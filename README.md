# Stereographic SEW Angle

Implementation of stereographic SEW (shoulder-elbow-wrist) angle for 7-DOF robot arms as well as inverse kinematics solutions for a number of 7-DOF arms from ["Redundancy parameterization and inverse kinematics of 7-DOF revolute manipulators"](https://www.sciencedirect.com/science/article/pii/S0094114X24002519).
The IK procedures also work with the conventional SEW angle.

Abstract: Seven-degree-of-freedom (DOF) robot arms have one redundant DOF for obstacle and singularity avoidance which must be parameterized to fully specify the joint angles for a given end effector pose. Commonly used 7-DOF revolute (7R) industrial manipulators from ABB, Motoman, and KUKA and space manipulators like SSRMS or FREND are conventionally parameterized by the shoulder–elbow–wrist (SEW) angle for path planning and teleoperation. We introduce the general SEW angle which generalizes the conventional SEW angle with an arbitrary reference direction function. Redundancy parameterizations such as the conventional SEW angle encounter an algorithmic singularity along a line in the workspace. We introduce a reference direction function choice called the stereographic SEW angle which has a singularity only along a half-line which can be out of reach, enlarging the usable workspace. We prove all parameterizations have an algorithmic singularity. Finally, using the general SEW angle and subproblem decomposition, we provide efficient singularity-robust inverse kinematics solutions which are often closed-form but may involve a 1D or 2D search. Search-based solutions may be converted to finding polynomial roots. Examples are available in a publicly accessible repository.

For examples using the polynomial method, see the [subproblem-polynomial](https://github.com/rpiRobotics/subproblem-polynomial) repo.

For 6-DOF IK solutions see [ik-geo](https://github.com/rpiRobotics/ik-geo). **Code in this repo depends on that code.**

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

`robot_examples`: Demonstration of IK for specific robots

`unit_tests`: Testing for smaller helper functions

`verify_eqns`: Verify equations used in paper


## Contributing

If you have any questions, improvements you'd like to make, or even ideas or requests for improvements, please start a GitHub issue or send an email.


# Tutorial: KUKA LBR iiwa 14 R820

Here we will demonstrate finding all IK solutions for the KUKA LBR iiwa 14 R820 using MATLAB. This robot falls into the R-2R-3R kinematic family and therefore has closed-form IK solutions.

You can follow along by running [robot_examples/kuka.m](robot_examples/kuka.m).

## Download and add to path
Other than MATLAB, you will need to download the following GitHub repos onto your local machine:

1. [stereo-sew](https://github.com/rpiRobotics/stereo-sew) (This repo, for 7-DOF IK code and SEW code)
2. [ik-geo](https://github.com/rpiRobotics/ik-geo) (For subproblem solution code)

Then, add the code the the [MATLAB path](https://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html).
Right click on each folder in MATLAB and click `Add To Path > Selected Folders and Subfolders`


## Define robot kinematics parameters

IK-Geo uses the product of exponentials convention to define the robot kinematic parameters.
(If you already have the Denavit-Hartenberg parameters, you can use [dh_to_kin.m](https://github.com/rpiRobotics/ik-geo/blob/main/matlab/robot_IK_helpers/dh_to_kin.m) to convert.)

Kinematic parameters for the  KUKA LBR iiwa 14 R820 are already defined in [+robot_kin/kuka.m](https://github.com/rpiRobotics/stereo-sew/blob/main/%2Brobot_kin/kuka.m). The columns of `kin.P`are $p_{i, i+1}$ and $p_{7T}$, the columns of `kin.H` are $h_{i}$, and `kin.joint_type` is all zeros because all joints are revolute.

```MATLAB
kin = robot_kin.kuka();
```

## Define desired end effector pose and SEW angle

The 6-DOF end effector pose is defined in this base by $R_{07}$ and $R_{0T}$.
The remaining 1 DOF is parameterized by the SEW angle. In this case we use the stereographic SEW angle $\pi/6$ with $e_t$ pointing in the negative $z$ direction and $e_r$ pointing in the positive $y$ direction.

```MATLAB
R_07 = rot([1;0;0], pi/4);
p_0T = [0.2; 0.3; 0.4];
SEW = sew_stereo([0;0;-1], [0;1;0]);
psi = pi/6;
```

## Perform inverse kinematics

We call the `IK_2R_2R_3R()` function using the kinematic parameters and desired 7-DOF pose. Each column of `Q` is a vector `q` of joint angles. Each element of `is_LS_vec` is 0 if the solution is exact and 1 if the solution is an approximate continuous solution.
(This is 1 for IK branches where the desired pose is on the boundary or outside of the workspace.)

```MATLAB
[Q, is_LS_vec] = SEW_IK.IK_2R_2R_3R(R_07, p_0T, SEW, psi, kin)
```

In this case, all 8 solutions are exact.

## Double check solutions using forward kinematics

If we take each `q` and use forward kinematics to find the resultant end effector pose and SEW angle, we see all IK solutions are correct. All errors are zero to a tolerance of less than `1.0e-15`.

```MATLAB
i = 1;
[R_07_t, p_0T_t, P_SEW] = fwdkin_inter(kin, Q(:,i), [1, 3, 5]);
psi_t = SEW.fwd_kin(P_SEW(:,1), P_SEW(:,2), P_SEW(:,3));

R_07_t - R_07
p_0T_t - p_0T
wrapToPi(psi_t - psi)
```
