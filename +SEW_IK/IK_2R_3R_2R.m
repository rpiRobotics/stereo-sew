function [Q, is_LS_vec] = IK_2R_3R_2R(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

% Use subproblem 3 to find theta_S
% || p_17 - R(n_SEW, theta) ||p_23|| e_17 || = || p_56 ||
p_17 = W-S;
e_17 = p_17/ norm(p_17);

[~, n_SEW] = SEW_class.inv_kin(S, W, psi);
[theta_S, theta_S_is_LS] = subproblem.sp_3(norm(kin.P(:,3))*e_17, p_17, n_SEW, norm(kin.P(:,6)));

% Pick theta_S > 0 so R_02 p_23 falls in the correct half plane
q_S = max(theta_S);

p_S_E = rot(n_SEW, q_S) * norm(kin.P(:,3))*e_17;

% Find q_1 and q_2 using subproblem 2
h_1 = kin.H(:,1);
h_2 = kin.H(:,2);
[t1, t2, t12_is_ls] = subproblem.sp_2(p_S_E,kin.P(:,3), -h_1, h_2);

for i_q12 = 1:length(t1)
    q1 = t1(i_q12);
    q2 = t2(i_q12);
    R_02 = rot(h_1, q1) * rot(h_2, q2);
    % Find q_6 and q_7 using subproblem 2
    [t7, t6, t67_is_ls] = subproblem.sp_2(R_07'*(p_17-R_02*kin.P(:,3)),kin.P(:,6), kin.H(:,7), -kin.H(:,6));

    for i_q67 = 1:length(t6)
        q6 = t6(i_q67);
        q7 = t7(i_q67);

        % Find q_2 and q_3 using subproblem 2
        R_57 = rot(kin.H(:,6), q6) * rot(kin.H(:,7), q7);
        [t3, t4, t34_is_ls] = subproblem.sp_2(R_02'*R_07*R_57'*kin.H(:,5), kin.H(:,5), -kin.H(:,3), kin.H(:,4));

        for i_q34 = 1:length(t3)
            q3 = t3(i_q34);
            q4 = t4(i_q34);

            % Find q_5
            R_24 = rot(kin.H(:,3), q3) * rot(kin.H(:,4), q4);
            p = kin.H(:,6); % Can't be collinear with h_5
            [q5, q5_is_ls] = subproblem.sp_1(p, R_24'*R_02'*R_07*R_57'*p, kin.H(:,5)); 

            q_i = [q1; q2; q3; q4; q5; q6; q7];
            Q = [Q q_i];
            % is_LS_vec = [is_LS_vec [theta_S_is_LS; t12_is_ls; t67_is_ls; t34_is_ls; q5_is_ls]];
            is_LS_vec = [is_LS_vec theta_S_is_LS||t12_is_ls||t67_is_ls||t34_is_ls||q5_is_ls];
        end
    end
end

end