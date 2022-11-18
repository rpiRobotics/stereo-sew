function [alignment, q_solns_partial] = q67_alignment_given_wrist_angle(wrist_angle, kin, R, T, psi, SEW)
if isfield(kin, 'RT')
    R = R * kin.RT';
end
q_solns_partial = [];
alignment = [];

p_W_EE_0 = kin.P(:,end);
p_02_0 = sum(kin.P(:,1:2),2);
h_1 = kin.H(:,1);

p_E2_0 = -kin.P(:,4);
d_2E = norm(p_E2_0);

p_WE_0 = -kin.P(:,6);
d_WE = norm(p_WE_0);


% Find wrist position
W = T - R * p_W_EE_0;

% Find shoulder position
S = kin.P(:,1);

p_17 = W-S;

% Calculate elbow postion
% Elbow lies on the half-circle in the SEW plane centered around the wrist
w_hat = vec_normalize(W - S);

[~, n_SEW] = SEW.inv_kin(S, W, psi);
p_WE = rot(-n_SEW, wrist_angle) * (-w_hat) * d_WE;
E = W + p_WE;

% Find the position of O_2 = O_3,
% which lies on the cone with axis h_1,
% and lies a distance of d_2E from the elbow E
q_1_solns = subproblem.sp_3(p_02_0, E, h_1, d_2E);

for q_1 = q_1_solns
    % Find q_2, q_3 with subproblem 2
    R_10 = rot(kin.H(:,1), -q_1);
    
%     global q_ap
%     R_12 = rot(kin.H(:,2), q_ap(2));
%     R_23 = rot(kin.H(:,3), q_ap(3));
%     R_23 * kin.P(:,4)
%     R_12' * (R_10*p_17 +R_10*p_WE-kin.P(:,2))

    [q_3_solns, q_2_solns, q_23_is_LS] = subproblem.sp_2(kin.P(:,4), R_10*p_17+R_10*p_WE-kin.P(:,2), kin.H(:,3), -kin.H(:,2));
    if q_23_is_LS
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
            continue
        end
        for i_45 = 1:length(q_4_solns)
            q_4 = q_4_solns(i_45);
            q_5 = q_5_solns(i_45);
            R_34 = rot(kin.H(:,4), q_4);
            R_45 = rot(kin.H(:,5), q_5);
            R_05 = R_10' * R_21' * R_32' * R_34 * R_45;
            e_i = kin.H(:,6)'* R_05' * R * kin.H(:,7) - kin.H(:,6)'*kin.H(:,7);
            alignment = [alignment e_i];
            q_solns_partial = [q_solns_partial [q_1; q_2; q_3; q_4; q_5]];
        end
    end
end

end


function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
