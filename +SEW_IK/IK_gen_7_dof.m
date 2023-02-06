function [Q, is_LS_vec] = IK_gen_7_dof(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

p_17 = W - S;
e_SW = p_17 / norm(p_17);

[e_CE, n_SEW] = SEW_class.inv_kin(S, W, psi);

[q1_vec, q2_vec, soln_num_vec] = search_2D(@q4_solvability_given_q12, -pi, pi, -pi, pi, 100, true);

for i = 1:length(q1_vec)
    [~, Q123_567] = q4_solvability_given_q12(q1_vec(i), q2_vec(i));
    [q, is_LS] = q_given_q123_567(Q123_567(:,soln_num_vec(i)));
    Q = [Q q];
    is_LS_vec = [is_LS_vec is_LS];
end

function [e, Q123_567] =  q4_solvability_given_q12(q1, q2)
    e = NaN(1, 8);
    Q123_567 = NaN(6, 8);
    i_soln = 1;

    R_01 = rot(kin.H(:,1), q1);
    R_12 = rot(kin.H(:,2), q2);
    R_02 = R_01 * R_12;

    % Find q3 with Subproblem 4
    [t3, t3_is_LS] = subproblem.sp_4(R_02'*n_SEW, kin.P(:,4), kin.H(:,3), -n_SEW'*(R_01*kin.P(:,2)+R_02*kin.P(:,3)));
    if t3_is_LS
        return
    end

    for i_3 = 1:length(t3)
        if i_3 == 2
            i_soln = 5;
        end

        q3 = t3(i_3);
        R_23 = rot(kin.H(:,3), q3);
        R_03 = R_02 * R_23;
        p_14 = R_01*kin.P(:,2) + R_02*kin.P(:,3) + R_03*kin.P(:,4);

        % Check for correct half plane
        if e_CE' * p_14 < 0
            continue
        end
        
        % Find (q5, q6, q7) with Subproblem 5
        [t7, t6, t5] = subproblem.sp_5( ...
            -kin.P(:,7), ...
            R_07'*(p_17-p_14), ...
            kin.P(:,6), ...
            kin.P(:,5), ...
            kin.H(:,7), ...
            -kin.H(:,6), ...
            -kin.H(:,5));
        for i_567 = 1:length(t5)
            q5 = t5(i_567); q6 = t6(i_567); q7 = t7(i_567);
             % Calculate alignment error
             R_45 = rot(kin.H(:,5), q5);
             R_56 = rot(kin.H(:,6), q6);
             R_67 = rot(kin.H(:,7), q7);
             R_47 = R_45 * R_56 * R_67;
             
             e_i = norm(R_03'*R_07*R_47'*kin.H(:,4) - kin.H(:,4));
             e(i_soln) = e_i;
             Q123_567(:,i_soln) = [q1; q2; q3; q5; q6; q7];
             i_soln = i_soln + 1;
        end
    end
end

function [q, is_LS] = q_given_q123_567(q123_567)
    R_01 = rot(kin.H(:,1), q123_567(1));
    R_12 = rot(kin.H(:,2), q123_567(2));
    R_23 = rot(kin.H(:,3), q123_567(3));

    R_45 = rot(kin.H(:,5), q123_567(4));
    R_56 = rot(kin.H(:,6), q123_567(5));
    R_67 = rot(kin.H(:,7), q123_567(6));

    R_03 = R_01*R_12*R_23;
    R_47 = R_45*R_56*R_67;

    p = kin.H(:,2); % Can't be collinear with h_1
    [q4, is_LS] = subproblem.sp_1(p, R_03'*R_07*R_47'*p, kin.H(:,4));
    q = [q123_567(1:3); q4; q123_567(4:6)];
end

end