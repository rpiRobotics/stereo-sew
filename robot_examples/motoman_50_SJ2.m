% Motoman SIA-50 with shoulder chosen at joint 2
% Notice that due to extra symmetry compared to the general R_2R_R_3R case,
% the 4 branches in the 1D search always come in equal pairs
% This means the search time can be cut in half


ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = zeros(3,1);

kin = SIA50D_search();
SEW = sew_conv(rot(ey,-pi/4)*ez);

%q = [0 -pi/4 0 pi/2 0 -pi/4 0];
% q = [0.1, -0.7, -0.2, +1.5, 0.15, -0.8, 0.1];
q = rand_angle([7 1]);

[R, T, P_SEW] = fwdkin_inter(kin, q, [2 4 5]);
p_S = P_SEW(:,1);
p_E = P_SEW(:,2);
p_W = P_SEW(:,3);

T;
R;
psi = SEW.fwd_kin(p_S, p_E, p_W);

[Q, is_LS] = SEW_IK.IK_R_2R_R_3R_SJ2(R, T, SEW, psi, kin, true);
%xline(q(1));
xlabel("q_1")
ylabel("SEW error")
% length(is_LS)
S.Q = Q;
P.kin = kin;
P.R = R;
P.T = T;
P.sew = SEW;
P.psi = psi;
[e, e_R, e_T, e_psi] = error(P, S)

%%
kin = robot_kin.SIA50D;

h_fig = diagrams.setup;

view(40,30)
hold on

for i = 1
diagrams.robot_plot(kin, Q(:,i), ...
    cyl_radius = 0.05, ...
    cyl_half_length = 0.1, ...
    unit_size = 0.1, ...
    show_arrows = true, ...
    show_arrow_labels = false, ...
    show_joint_labels = false, ...
    show_base_label = false, ...
    show_task_label = false);
end
hold off
diagrams.redraw()
%%

function [e, e_R, e_T, e_psi] = error(P, S)
    e_R = NaN([1 width(S.Q)]);
    e_T = NaN([1 width(S.Q)]);
    e_psi = NaN([1 width(S.Q)]);
    
    for i = 1:width(S.Q)
        [R_t, T_t, P_SEW_t] = fwdkin_inter(P.kin, S.Q(:,i), [2 4 5]);
        e_R(i) = norm(R_t - P.R);
        e_T(i) = norm(T_t - P.T);
        
        psi_t = P.sew.fwd_kin(P_SEW_t(:,1),P_SEW_t(:,2),P_SEW_t(:,3));
        e_psi(i) = norm(psi_t - P.psi);
    end
    e = e_R + e_T + e_psi;
end

function kin = SIA50D_search
    % https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/sia-series/sia50d

    d1 = 0.540;
    a1 = 0.145;
    d3 = 0.875;
    d5 = 0.610;
    dT = 0.350;

    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = zeros(3,1);

    kin.P = [d1*ez a1*ex zv d3*ez d5*ez zv zv dT*ez];
    kin.H = [ez -ey ez -ey ez -ey ez];
    kin.joint_type = [0 0 0 0 0 0 0];
    
end