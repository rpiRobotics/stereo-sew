P.kin = robot_kin.spacebot;

% Pick a random rotation and position for the end effector
P.R = eye(3);
P.T = rand_vec;

% Pick a random SEW angle, and use the conventional SEW angle definition
P.sew = sew_conv([1;0;0]);
P.psi = rand_angle;

% Compute all inverse kinematics solutions
N_trial = 1000;
tic
for i = 1:N_trial
[S.Q, S.is_LS] = SEW_IK.IK_2R_2R_3R(P.R, P.T, P.sew, P.psi, P.kin);
end
T = toc/N_trial;
f = 1/T;

% Find error, which should be zero for non-least-squares solutions
[e, e_R, e_T, e_psi] = SEW_IK_setups.IK_2R_2R_3R.error(P, S);

clc
disp("Average time to compute all IK solutions: " + T*1e6 + " us (" + f + " Hz)")
disp("Errrors for non-least-squares solutions:")
disp(e(~S.is_LS))