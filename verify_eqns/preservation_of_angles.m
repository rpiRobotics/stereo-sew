e_SW = rand_normal_vec;
e_r_1 = rand_normal_vec;
e_t = rand_perp_normal_vec(e_r_1);

alpha = rand_angle
e_r_2 = rot(e_t, alpha) * e_r_1;


e_x_1 = e_x(e_SW, e_t, e_r_1);
e_x_2 = e_x(e_SW, e_t, e_r_2);
[beta, is_LS] = subproblem.sp_1(e_x_1, e_x_2, -e_SW)

%%
e_r = rand_normal_vec;
e_t = rand_perp_normal_vec(e_r);
alpha = rand_angle;
p_SW = rand_vec;
e_SW = p_SW / norm(p_SW);

normalize( cross(p_SW, rot(e_t, -alpha)*e_r) )
normalize( rot(e_SW, alpha) * cross(p_SW, e_r) )

% n1 = norm(cross(p_SW, rot(e_t, -alpha)*e_r))
% n2 = norm(rot(e_SW, alpha) * cross(p_SW, e_r))


%%

kin = robot_kin.SIA5D;
%SEW_conv = sew_conv([0;0;1]);
SEW_conv = sew_stereo(-[0;0;1], [1;0;0]);
q=rand_angle(7);

[P.R, P.T, P_SEW] = fwdkin_inter(kin, q, [1 4 5]);
[J_psi_e, J_psi_w] = SEW_conv.jacobian(P_SEW(:,1), P_SEW(:,2), P_SEW(:,3));
kin_E = kin;
kin_E.joint_type = [0 0 0];
kin_W = kin;
kin_W.joint_type = [0 0 0 0 0];
J_E = robotjacobian(kin_E, q);
J_E = [J_E(4:6, :) zeros(3, 4)];
J_W = robotjacobian(kin_W, q);
J_W = [J_W(4:6, :) zeros(3,2)];

J_psi = J_psi_e*J_E + J_psi_w*J_W

function vn = normalize(v)
    vn = v / norm(v)
end

function e_x = e_x(e_SW, e_t, e_r)
    k_r = cross(e_SW - e_t, e_r);
    k_x = cross(k_r, e_SW);
    e_x = k_x / norm(k_x);
end

