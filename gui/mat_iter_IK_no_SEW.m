function [qq, solInfo] = mat_iter_IK_no_SEW(R, T, GC, robot, body_names, q_guess)
% body_names
%   .EE
%   .S
%   .E
%   .W

% GC
%   [GC_2 GC_4 GC_6]
%   q_i has the sign of GC_i
%   GC_i = 1 if q_i = 0

persistent gik
if isempty(gik)
    params.EnforceJointLimits  = false;
    gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'pose', 'joint'}, 'SolverParameters', params);
end

% End effector constraint
pose_const = constraintPoseTarget(body_names.EE,'TargetTransform', trvec2tform(T')*rotm2tform(R));

% GC constraint
GC_bounds = [-inf inf
             -inf inf
             -inf inf
             -inf inf
             -inf inf
             -inf inf
             -inf inf
             -inf inf];
if GC(1) == 1
    GC_bounds(2+1, 1) = 0;
else
    GC_bounds(2+1, 2) = -eps;
end

if GC(2) == 1
    GC_bounds(4+1, 1) = 0;
else
    GC_bounds(4+1, 2) = -eps;
end

if GC(3) == 1
    GC_bounds(6+1, 1) = 0;
else
    GC_bounds(6+1, 2) = -eps;
end


joint_const = constraintJointBounds(robot,'Bounds',GC_bounds);

[configSol,solInfo] = gik(q_guess, pose_const, joint_const);

qq = configSol;

end
