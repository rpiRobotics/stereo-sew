function [Q, is_LS_vec] = IK_3R_R_2R_R(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

p_17 = W - S;
e_SW = p_17 / norm(p_17);

[~, n_SEW] = SEW_class.inv_kin(S, W, psi);

%e_fun = @(TS)(q4_solvability_given_TS(TS));
[TS_vec, soln_num_vec] = search_1D(@q4_solvability_given_TS, 0, pi, 10000, true);

for i = 1:length(TS_vec)
    [~, q_solns_partial_i] = q4_solvability_given_TS(TS_vec(i));
    q_partial_i = q_solns_partial_i(:,soln_num_vec(i));
    [q, is_LS] = q_given_q567(q_partial_i, TS_vec(i));
    Q = [Q q];
    is_LS_vec = [is_LS_vec is_LS];
end


function [e, q_solns_partial] = q4_solvability_given_TS(theta_S)
    q_solns_partial = NaN(3, 4);
    e = NaN(1, 4);
    i_soln = 1;

    p_14 = rot(n_SEW, theta_S)*e_SW*norm(kin.P(:,4));
    % Use subproblem 3 to find q_7
    [t7, t7_is_LS] = subproblem.sp_3(kin.P(:,7), R_07'*(p_17-p_14), -kin.H(:,7), norm(kin.P(:,5)));
    if t7_is_LS
        return
    end
    
    for i_7 = 1:length(t7)
        q7 = t7(i_7);
        R_67 = rot(kin.H(:,7), q7);
    
        % Use subproblem 2 to find (q_5, q_6)
        [t5, t6, t56_is_LS] = subproblem.sp_2(kin.P(:,5), R_67*R_07'*(p_17-p_14)-kin.P(:,7), -kin.H(:,5), kin.H(:,6));
        if t56_is_LS
            i_soln = i_soln + 2;
            continue
        end
    
        for i_56 = 1:length(t5)
            q5 = t5(i_56);
            q6 = t6(i_56);
            R_45 = rot(kin.H(:,5), q5);
            R_56 = rot(kin.H(:,6), q6);
            R_47 = R_45 * R_56 * R_67;
            
            % Find Subproblem 1 solvability error for q_4
            e(i_soln) = kin.H(:,4)' * (kin.P(:,4) - R_47*R_07'*p_14);
            q_solns_partial(:,i_soln) = [q5; q6; q7];
            i_soln = i_soln+1;
        end
    end
end

function [Q, is_LS_vec] = q_given_q567(q567, theta_S)
    Q = [];
    is_LS_vec = [];

    q5 = q567(1); q6 = q567(2); q7 = q567(3);
    p_14 = rot(n_SEW, theta_S)*e_SW*norm(kin.P(:,4));
    R_45 = rot(kin.H(:,5), q5);
    R_56 = rot(kin.H(:,6), q6); 
    R_67 = rot(kin.H(:,7), q7);
    R_47 = R_45 * R_56 * R_67;
    % Find q4 using subproblem 1
    [q4, q4_is_LS] = subproblem.sp_1(kin.P(:,4), R_47*R_07'*p_14, -kin.H(:,4));
    R_34 = rot(kin.H(:,4), q4);
    R_03 = R_07 * R_47' * R_34';

    % Find (q_1, q_2, q_3) by solving the spherical shoulder
    [t2, t1, t12_is_LS] = subproblem.sp_2(kin.H(:,3), R_03*kin.H(:,3), kin.H(:,2), -kin.H(:,1));
    
    for i_12 = 1:length(t1)
        q1 = t1(i_12);
        q2 = t2(i_12);
        R_01 = rot(kin.H(:,1), q1);
        R_12 = rot(kin.H(:,2), q2);
        p = kin.H(:,2); % can't be collinear with h_3
        [q3, q3_is_LS] = subproblem.sp_1(p, R_12'*R_01'*R_03*p, kin.H(:,3));
        
        Q = [Q [q1; q2; q3; q4; q567]];
        is_LS_vec = [is_LS_vec q4_is_LS || t12_is_LS || q3_is_LS];
    end
end

end