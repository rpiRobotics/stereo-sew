function [Q, is_LS_vec] = IK_3R_R_3R(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

p_SW = W-S;
e_SW = p_SW / norm(p_SW);

[~, n_SEW] = SEW_class.inv_kin(S, W, psi);

% Use subproblem 3 to find q4
[t4, t4_is_LS] = subproblem.sp_3(kin.P(:,5), -kin.P(:,4), kin.H(:,4), norm(p_SW));

for i_4 = 1:length(t4)
    q4 = t4(i_4);

    % Solve for theta_b, theta_c using subproblem 2
    % TODO only keep one solution, as they will represent the same R_03
    [t_b, t_c, t_bc_is_LS] = subproblem.sp_2(p_SW, kin.P(:,4)+rot(kin.H(:,4),q4)*kin.P(:,5), -n_SEW,e_SW);
    for i_bc = 1:length(t_b)
        theta_b = t_b(i_bc);
        theta_c = t_c(i_bc);

        % Solve for theta_a using subproblem 4
        % Keep only solutions that put the elbow in the correct half plane
        [t_a, t_a_is_LS] = subproblem.sp_4(n_SEW, rot(n_SEW, theta_b)*rot(e_SW, theta_c)*kin.P(:,4), e_SW, 0);
        for i_a = 1:length(t_a)
            theta_a = t_a(i_a);
            if theta_a > 0
                continue
            end

            % Find q_1, q_2, q_3 with subproblems 2 and 1
            R_03 = rot(e_SW, theta_a)*rot(n_SEW, theta_b)*rot(e_SW, theta_c);
            p_spherical_1 = kin.H(:, 2); % Must be noncollinear with h_3
            [t2, t1, t12_is_LS] = subproblem.sp_2(kin.H(:,3), R_03*kin.H(:,3), kin.H(:,2), -kin.H(:,1));
            for i_12 = 1:length(t1)
                q1 = t1(i_12);
                q2 = t2(i_12);
                [q3, q3_is_LS] = subproblem.sp_1(p_spherical_1, (rot(kin.H(:,1), q1) * rot(kin.H(:,2), q2))'*R_03*p_spherical_1, kin.H(:,3));

                % Find q_5, q_6, q_7 with subproblems 2 and 1
                R_01 = rot(kin.H(:,1), q1);
                R_12 = rot(kin.H(:,2), q2);
                R_23 = rot(kin.H(:,3), q3);
                R_34 = rot(kin.H(:,4), q4);
                R_47 = (R_01 * R_12 * R_23 * R_34)' * R_07;
                p_spherical_2 = kin.H(:, 6); % Must be noncollinear with h_7
                [t6, t5, t56_is_LS] = subproblem.sp_2(kin.H(:,7), R_47*kin.H(:,7), kin.H(:,6), -kin.H(:,5));
                for i_56 = 1:length(t5)
                    q5 = t5(i_56);
                    q6 = t6(i_56);
                    [q7, q7_is_LS] = subproblem.sp_1(p_spherical_2, (rot(kin.H(:,5), q5) * rot(kin.H(:,6), q6))'*R_47*p_spherical_2, kin.H(:,7));
                    q_i = [q1; q2; q3; q4; q5; q6; q7];
                    
                    Q = [Q q_i];
                    % is_LS_vec = [is_LS_vec [t4_is_LS;t_bc_is_LS;t_a_is_LS;t12_is_LS;q3_is_LS;t56_is_LS;q7_is_LS]];
                    is_LS_vec = [is_LS_vec [t4_is_LS||t_bc_is_LS||t_a_is_LS||t12_is_LS||q3_is_LS||t56_is_LS||q7_is_LS]];
                end
            end

            
        end

    end
end

end