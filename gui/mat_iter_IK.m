function [qq, SEW_axes_at_S, solInfo] = mat_iter_IK(R, T, SEW_type, psi, GC, robot, body_names, S, W, q_guess)
% body_names
%   .EE
%   .S
%   .E
%   .W

% GC
%   [GC_2 GC_4 GC_6]
%   q_i has the sign of GC_i
%   GC_i = 1 if q_i = 0

%persistent gik
%if isempty(gik)
    params.EnforceJointLimits  = false;
    params.MaxIterations = 2e3;
    % 'SolverAlgorithm','LevenbergMarquardt',
    %gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'pose', 'joint', 'cartesian'}, 'SolverParameters', params);
    gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'pose', 'joint'}, 'SolverParameters', params);
%end

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

% Elbow constraint

% half_plane_bounds = [0 inf
%                      0 0
%                      -inf inf];

half_space_bounds_1 = [0 inf
                       0 inf
                     -inf inf];
half_space_bounds_2 = [0 inf
                     -inf inf
                     -inf inf];

W_hat = vec_normalize(W - S);

if SEW_type == "Conventional"
    V = [1;0;0]; % TODO

    y_c = cross(W_hat, V);
    y_c = y_c / norm(y_c);

    x_c = cross(y_c, W_hat);

elseif SEW_type == "Stereographic"
    V_sew = [1;0;0];
    R_sew = [0;0;-1];
    
    n_hat_ref = vec_normalize(cross(W_hat - R_sew, V_sew));
    x_c = vec_normalize(cross(n_hat_ref, W_hat));
    y_c = cross(W_hat, x_c);
else
    error("Wrong SEW_type")
end

x_phi = x_c * cos(psi) + y_c * sin(psi);
z_phi = W_hat;
y_phi = cross(z_phi, x_phi);
SEW_axes_at_S = [x_phi y_phi z_phi];

cart_const_1 = constraintCartesianBounds(body_names.E, ...
    'ReferenceBody', body_names.S, ...
    'TargetTransform', rotm2tform(SEW_axes_at_S), ...
    'bounds', half_space_bounds_1);

cart_const_2 = constraintCartesianBounds(body_names.E, ...
    'ReferenceBody', body_names.S, ...
    'TargetTransform', rotm2tform(SEW_axes_at_S), ...
    'bounds', half_space_bounds_2);

[configSol,solInfo] = gik(q_guess, pose_const, joint_const);

qq = configSol;

end

function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
