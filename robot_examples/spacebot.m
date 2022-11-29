P.kin = robot_kin.spacebot;


P.R = eye(3);
%P.T = 1e-2*[1;2;3];
P.T = rand_vec;

P.sew = sew_conv([1;0;0]);
P.psi = deg2rad(45);

N_trial = 1000;
tic
for i = 1:N_trial
[S.Q, S.is_LS] = IK.IK_2R_2R_3R_mex(P.R, P.T, P.sew, P.psi, P.kin);
end
T = toc/N_trial
f = 1/T
[e, e_R, e_T, e_psi] = IK_setups.IK_2R_2R_3R.error(P, S)
