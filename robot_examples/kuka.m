% Demo of IK for KUKA LBR iiwa 14 R820
% Make sure to have both stereo-sew and ik-geo repos added to path

%% Define robot kinematic parameters

kin = robot_kin.kuka();

%% Define desired end effector pose and SEW angle

R_07 = rot([1;0;0], pi/4);
p_0T = [0.2; 0.3; 0.4];
SEW = sew_stereo([0;0;-1], [0;1;0]);
psi = pi/6;
%% Perform inverse kinematics

[Q, is_LS_vec] = SEW_IK.IK_2R_2R_3R(R_07, p_0T, SEW, psi, kin)
%% Double check solutions using forward kinematics

i = 1;
[R_07_t, p_0T_t, P_SEW] = fwdkin_inter(kin, Q(:,i), [1, 3, 5]);
psi_t = SEW.fwd_kin(P_SEW(:,1), P_SEW(:,2), P_SEW(:,3));

R_07_t - R_07
p_0T_t - p_0T
wrapToPi(psi_t - psi)