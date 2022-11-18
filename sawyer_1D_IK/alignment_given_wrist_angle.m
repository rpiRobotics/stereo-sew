function [alignment, q_solns] = alignment_given_wrist_angle(wrist_angle, kin, R, T, psi, SEW)
if isfield(kin, 'RT')
    R = R * kin.RT';
end
q_solns = [];
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
    p_2 = rot(h_1, q_1)*p_02_0;
    
    % apply IK starting from the EE and moving towards the base
    [q_7_solns, q_6_solns] = subproblem.sp_2(p_WE,R*p_WE_0, R*kin.H(:,7), -R*kin.H(:,6));
    

    for i_67 = 1:length(q_6_solns)
       q_7 =  q_7_solns(i_67);
       q_6 =  q_6_solns(i_67);
       R_6 = R * rot(-kin.H(:,7), q_7) * rot(-kin.H(:,6), q_6);
       [q_5_solns, q_4_solns] = subproblem.sp_2(p_2 - E, R_6*p_E2_0, R_6*kin.H(:,5), -R_6*kin.H(:,4));

        for i_45 = 1:length(q_4_solns)
            q_5 =  q_5_solns(i_45);
            q_4 =  q_4_solns(i_45);
            R_4 = R_6 * rot(-kin.H(:,5), q_5) * rot(-kin.H(:,4), q_4);
            [q_3_solns, q_2_solns] = subproblem.sp_2(-p_2,R_4*-p_02_0, R_4*kin.H(:,3), -R_4*kin.H(:,2));

            for i_23 = 1:length(q_2_solns)
                q_2 = q_2_solns(i_23);
                q_3 = q_3_solns(i_23);
                q_solns_i = [q_1 q_2 q_3 q_4 q_5 q_6 q_7]';
                q_solns = [q_solns q_solns_i];

                R_2 = R_4 * rot(-kin.H(:,3), q_3) * rot(-kin.H(:,2), q_2);
                alignment = [alignment dot(R_2*h_1, h_1)];
            end
        end
    end

end

end


function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
