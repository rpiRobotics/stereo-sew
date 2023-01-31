q = [   2.1408
       -1.5439
        1.9747
       -1.6115
        2.6971
       -0.9426
       -1.9063];

kin = robot_kin.sawyer;

sew = sew_conv([0;0;1]);

[R, T, P_SEW] = fwdkin_inter(kin, q, [1 4 6]);
psi = sew.fwd_kin(P_SEW(:,1),P_SEW(:,2),P_SEW(:,3));

[Q, is_LS_vec] = SEW_IK.IK_R_2R_2R_2R(R, T, sew, psi, kin)

%%

h_fig = diagrams.setup;

view(30,20)
hold on

i = 6;

diagrams.robot_plot(kin, Q(:,i), ...
    cyl_radius = 0.05, ...
    cyl_half_length = 0.1, ...
    unit_size = 0.1, ...
    show_arrows = true, ...
    show_arrow_labels = false, ...
    show_joint_labels = false, ...
    show_base_label = false, ...
    show_task_label = false);

hold off
diagrams.redraw()
%%
diagrams.save(h_fig, "sawyer_IK_"+i)