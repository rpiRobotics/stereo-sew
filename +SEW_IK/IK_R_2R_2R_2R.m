function [Q, is_LS_vec] = IK_R_2R_2R_2R(R_07, p_0T, SEW_class, psi, kin, show_plot)
if nargin < 6
    show_plot = false;
end

Q = [];
is_LS_vec = [];

e_fun = @(WA)(q67_alignment_given_wrist_angle(WA, kin, R_07, p_0T, psi, SEW_class));
[WA_vec, soln_num_vec] = search_1D(e_fun, -pi, 0, 200, show_plot);

for i = 1:length(WA_vec)
    [~, q_solns_partial_i] = q67_alignment_given_wrist_angle(WA_vec(i), kin, R_07, p_0T, psi, SEW_class);
    q_partial_i = q_solns_partial_i(:,soln_num_vec(i));
    [q, is_LS] = q_given_q12345(q_partial_i, kin, R_07);
    Q = [Q q];
    is_LS_vec = [is_LS_vec is_LS];
end

end

function [alignment, q_solns_partial] = q67_alignment_given_wrist_angle(wrist_angle, kin, R_07, T, psi, SEW)
    q_solns_partial = NaN(5, 8);
    alignment = NaN(1, 8);
    i_soln = 1;
    
    p_W_EE_0 = kin.P(:,8);
    h_1 = kin.H(:,1);
    
    p_E2_0 = -kin.P(:,4);
    d_2E = norm(p_E2_0);
    
    p_WE_0 = -kin.P(:,6);
    d_WE = norm(p_WE_0);
    
    
    % Find wrist position
    W = T - R_07 * p_W_EE_0;
    
    % Find shoulder position
    S = kin.P(:,1);
    
    p_17 = W-S;
    
    % Calculate elbow postion
    % Elbow lies on the half-circle in the SEW plane centered around the wrist
    w_hat = vec_normalize(W - S);
    
    [~, n_SEW] = SEW.inv_kin(S, W, psi);
    p_WE = rot(n_SEW, wrist_angle) * (-w_hat) * d_WE;
    
    % Find the position of O_2 = O_3,
    % which lies on the cone with axis h_1,
    % and lies a distance of d_2E from the elbow E
    [q_1_solns, q_1_is_LS] = subproblem.sp_3(kin.P(:,2), p_17+p_WE, h_1, d_2E);
    if q_1_is_LS
        return
    end
    
    for q_1 = q_1_solns
        % Find q_2, q_3 with subproblem 2
        R_10 = rot(kin.H(:,1), -q_1);
    
        [q_3_solns, q_2_solns, q_23_is_LS] = subproblem.sp_2(kin.P(:,4), R_10*p_17+R_10*p_WE-kin.P(:,2), kin.H(:,3), -kin.H(:,2));
        if q_23_is_LS
            i_soln = i_soln + 4;
            continue
        end
        for i_23 = 1:length(q_2_solns)
            q_2 = q_2_solns(i_23);
            q_3 = q_3_solns(i_23);
            R_21 = rot(kin.H(:,2), -q_2);
            R_32 = rot(kin.H(:,3), -q_3);
    
    
            % Find q_3, q_4 with subproblem 2
            [q_5_solns, q_4_solns, q_45_is_LS] = subproblem.sp_2(kin.P(:,6), R_32*R_21*(R_10*p_17-kin.P(:,2))-kin.P(:,4), kin.H(:,5), -kin.H(:,4));
            if q_45_is_LS
                i_soln = i_soln + 2;
                continue
            end
            for i_45 = 1:length(q_4_solns)
                q_4 = q_4_solns(i_45);
                q_5 = q_5_solns(i_45);
                R_34 = rot(kin.H(:,4), q_4);
                R_45 = rot(kin.H(:,5), q_5);
                R_05 = R_10' * R_21' * R_32' * R_34 * R_45;
                e_i = kin.H(:,6)'* R_05' * R_07 * kin.H(:,7) - kin.H(:,6)'*kin.H(:,7);
                alignment(i_soln) = e_i;
                q_solns_partial(:,i_soln) = [q_1; q_2; q_3; q_4; q_5];
                i_soln = i_soln + 1;
            end
        end
    end

end

function [q, is_LS] = q_given_q12345(q12345, kin, R_07)    
    R_01 = rot(kin.H(:,1), q12345(1));
    R_12 = rot(kin.H(:,2), q12345(2));
    R_23 = rot(kin.H(:,3), q12345(3));
    R_34 = rot(kin.H(:,4), q12345(4));
    R_45 = rot(kin.H(:,5), q12345(5));
    R_05 = R_01*R_12*R_23*R_34*R_45;
    
    [q6, q6_is_LS] = subproblem.sp_1(kin.H(:,7), R_05'*R_07*kin.H(:,7),  kin.H(:,6));
    [q7, q7_is_LS] = subproblem.sp_1(kin.H(:,6), R_07'*R_05*kin.H(:,6), -kin.H(:,7));
    
    q = [q12345; q6; q7];
    is_LS = q6_is_LS || q7_is_LS;
end


function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
