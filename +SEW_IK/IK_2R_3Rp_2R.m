function [Q, is_LS_vec] = IK_2R_3Rp_2R(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

p_17 = W - S;
e_SW = p_17 / norm(p_17);

[~, n_SEW] = SEW_class.inv_kin(S, W, psi);

% Find theta_h using Subproblem 4
[t_h, t_h_is_lS] = subproblem.sp_4(e_SW, p_17, -n_SEW, dot(kin.H(:,3), sum(kin.P(:,3:6),2)));

for i_h = 1:length(t_h)
    if t_h(i_h) < 0
        continue
    end

    % Find (q_1, q_2) using Subproblem 2
    [t2, t1, t12_is_LS] = subproblem.sp_2(kin.H(:,3), rot(n_SEW, t_h(i_h))*e_SW, kin.H(:,2), -kin.H(:,1));


    for i_12 = 1:length(t1)
        q1 = t1(i_12);
        q2 = t2(i_12);

        R_01 = rot(kin.H(:,1), q1);
        R_12 = rot(kin.H(:,2), q2);
        R_02 = R_01 * R_12;
        
        % solve (q_3+q_4+q_5, q_6, q_7) by solving a spherical wrist
        % First (q_3+q_4+q_5, q_6) using Subproblem 2...
        [t6, t345, t3456_is_LS] = subproblem.sp_2(kin.H(:,7), R_02'*R_07*kin.H(:,7), kin.H(:,6), -kin.H(:,3));
        
        for i_3456 = 1:length(t6)
            q6 = t6(i_3456);
            q345 = t345(i_3456);
            R_25 = rot(kin.H(:,3), q345);
            R_56 = rot(kin.H(:,6), q6);

            % ... and then q7 using Subproblem 1
            p = kin.H(:,6); % TODO: p must be non-collinear with h_7
            [q7, q7_is_LS] = subproblem.sp_1(p, R_56' * R_25' * R_02' * R_07 * p, kin.H(:,7));

            % Solve (q3, q4, q5)
            p = R_02' * p_17 - kin.P(:,3) - R_25 * kin.P(:,6);
            [t4, t4_is_LS] = subproblem.sp_3(kin.P(:,5), -kin.P(:,4), kin.H(:,4), norm(p));
            for i_4 = 1:length(t4)
                q4 = t4(i_4);
                [q3, q3_is_LS] = subproblem.sp_1(kin.P(:,4) + rot(kin.H(:,4),q4)*kin.P(:,5), p, kin.H(:,3));

                % Find q5 with subtraction
                q5 = wrapToPi(q345 - q3 - q4);

                q_i = [q1 q2 q3 q4 q5 q6 q7]';
                Q = [Q q_i];

                %is_LS_vec = [is_LS_vec [t_h_is_lS; t12_is_LS; t3456_is_LS; q7_is_LS; t4_is_LS; q3_is_LS]];
                is_LS_vec = [is_LS_vec [t_h_is_lS || t12_is_LS || t3456_is_LS || q7_is_LS || t4_is_LS || q3_is_LS]]
            end
        end
    end
end
end