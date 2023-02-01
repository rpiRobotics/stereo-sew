%% Plot self-motion manifold
kin = robot_kin.SIA50D;

for trial = 1:1e5
Q = [];
psi_vec = linspace(0,2*pi, 200);
R = eye(3);
%k_R = rand_normal_vec;
%theta_R = rand_angle;
%R  = rot(k_R, theta_R);
%T = rand_vec*3;
T = [0.9;0.3; 1.4]
done = true;

for i = 1:length(psi_vec)
    psi = psi_vec(i);
    %[Q_i, is_LS_vec] = SEW_IK.IK_R_2R_2R_2R(R, T, sew, psi, kin);
    [Q_i, is_LS_vec] = SEW_IK.IK_R_R_3R_2R(R, T, sew, psi, kin);
    assert(~isempty(Q_i))
%     if (isempty(Q_i))
%         done = false;
%         break
%     end

    if i == 1
        Q = Q_i(:,1);
    else
        [Q(:,i), ~, diff_norm] = closest_q(Q_i, Q(:,i-1));
        assert(diff_norm < 5)
%         if diff_norm > 2
%             done = false;
%             break
%         end
    end
end
if done
    disp("Done!" + trial)
    k_R
    theta_R
    T
    %plot(Q', 'x')
    break
end

end



P_E_mat = [];
for i = 1:length(psi_vec)
    [~, ~, P_E_i] = fwdkin_inter(kin, Q(:,i), 4);
    P_E_mat(:,i) = P_E_i;
end



h_fig = diagrams.setup;

view(30,20)
hold on
diagrams.utils.plot3_mat(P_E_mat);
i = 1;

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
diagrams.save(h_fig, "manifold")