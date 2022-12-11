function [Q, is_LS_vec] = IK_R_R_3R_2R(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

p_17 = W - S;
e_SW = p_17 / norm(p_17);

% Find (q_1, q_2, theta_W) with Subproblem 5
% Only keep theta_W <= 0
[~, n_SEW] = SEW_class.inv_kin(S, W, psi);
[tW, t1, t2] = subproblem.sp_5(p_17, -e_SW * norm(kin.P(:,6)),kin.P(:,2), kin.P(:,3), n_SEW, kin.H(:,1), kin.H(:,2));
for i_q12 = 1:length(t1)
    qW = tW(i_q12);
    if qW > 0
        continue
    end
    q1 = t1(i_q12);
    q2 = t2(i_q12);

    % Find (q_6, q_7) with Subproblem 2
    [t7, t6, t67_is_LS] = subproblem.sp_2( ...
        R_07' * rot(n_SEW, qW)*e_SW* norm(kin.P(:,6)), kin.P(:,6), kin.H(:,7), -kin.H(:,6));
    
    for i_67 = 1:length(t6)
        q6 = t6(i_67);
        q7 = t7(i_67);
        % Find (q_3, q_4) with Subproblem 2
        
        R_01 = rot(kin.H(:,1), q1);
        R_12 = rot(kin.H(:,2), q2);
        R_56 = rot(kin.H(:,6), q6);
        R_67 = rot(kin.H(:,7), q7);
        R_25 = (R_01 * R_12)' * R_07 * (R_56 * R_67)';

        [t4, t3, t34_is_LS] = subproblem.sp_2(kin.H(:,5), R_25*kin.H(:,5), kin.H(:,4), -kin.H(:,3));
        
        for i_34 = 1:length(t3)
            q3 = t3(i_34);
            q4 = t4(i_34);
            % Find q_5 with Subproblem 1
            p = kin.H(:,6); % TODO: p has to be non-collinear with h_5
            R_23 = rot(kin.H(:,3), q3);
            R_34 = rot(kin.H(:,4), q4);
            [q5, q5_is_LS] = subproblem.sp_1(p, (R_23 * R_34)'*R_25*p, kin.H(:,5));

            q_i = [q1 q2 q3 q4 q5 q6 q7]';
            Q = [Q q_i];
            %is_LS_vec = [is_LS_vec [t67_is_LS; t34_is_LS; q5_is_LS]];
            is_LS_vec = [is_LS_vec [t67_is_LS || t34_is_LS || q5_is_LS]];
        end
    end
end


