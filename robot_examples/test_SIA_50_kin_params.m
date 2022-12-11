ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = zeros(3,1);

kin = robot_kin.SIA50D();
SEW = sew_conv(rot(ey,-pi/4)*ez);

% q = [0 -pi/4 0 pi/2 0 -pi/4 0];
q = [0.1, -0.7, -0.2, +1.5, 0.15, -0.8, 0.1];

[R, T, P_SEW] = fwdkin_inter(kin, q, [1 3 6]);
S = P_SEW(:,1);
E = P_SEW(:,2);
W = P_SEW(:,3);

T
R
psi = SEW.fwd_kin(S, E, W)

[Q, is_LS] = SEW_IK.IK_R_R_3R_2R(R, T, SEW, psi, kin);
%%

for i = 1:1e4
    R = rot(rand_normal_vec, rand_angle);
    T = rand_vec;
    psi = rand_angle;
    [Q, is_LS] = SEW_IK.IK_R_R_3R_2R(R, T, SEW, psi, kin);
    if length(is_LS) > 8
        disp(Q);
        break
    end
end