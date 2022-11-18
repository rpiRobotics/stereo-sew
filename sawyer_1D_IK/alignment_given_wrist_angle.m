function [alignment, q_solns] = alignment_given_wrist_angle(wrist_angle, kin, R, T, psi, V)
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
n_ref = vec_normalize(cross(w_hat, V));
n_ref_perp = cross(w_hat, n_ref);
n_SEW = cos(psi)*n_ref +...
        sin(psi)*n_ref_perp;

WA_X = -w_hat;
WA_Y = cross(n_SEW, w_hat);

p_WE =   cos(wrist_angle)*WA_X*d_WE...
       + sin(wrist_angle)*WA_Y*d_WE;
E = W + p_WE;

% Find the position of O_2 = O_3,
% which lies on the cone with axis h_1,
% and lies a distance of d_2E from the elbow E



% theta = subproblem3_linear(p1, p2, k, d)
% || p2 - rot(k, theta)*p1 || = d
%q_1_solns = subproblem3_linear(p_02_0, E, kin.H(:,1), d_2E)

%   theta = subproblem3(p, q, k, d)
%   
%   solve for theta in an elbow joint according to
%   || q - rot(k, theta)*p || = d
q_1_solns = fixed_subproblem3(p_02_0, E, h_1, d_2E);
q_1_solns = wrapToPi(q_1_solns);

%for i = length(q_1_solns)
%    q_1 = q_1_solns(i);
for q_1 = q_1_solns'
    p_2 = rot(h_1, q_1)*p_02_0;
    
    % apply IK starting from the EE and moving towards the base
    %   [theta1, theta2] = subproblem2(p, q, k1, k2)
    %   q = rot(k1, theta1) * rot(k2, theta2) * p
    [q_7_solns, q_6_solns] = subproblem2(R*p_WE_0, p_WE, -R*kin.H(:,7), -R*kin.H(:,6));
    q_7_solns = wrapToPi(q_7_solns);
    q_6_solns = wrapToPi(q_6_solns);

    for i_67 = 1:length(q_6_solns)
       q_7 =  q_7_solns(i_67);
       q_6 =  q_6_solns(i_67);
       R_6 = R * rot(-kin.H(:,7), q_7) * rot(-kin.H(:,6), q_6);
       [q_5_solns, q_4_solns] = subproblem2(R_6*p_E2_0, p_2 - E, -R_6*kin.H(:,5), -R_6*kin.H(:,4));
        q_5_solns = wrapToPi(q_5_solns);
        q_4_solns = wrapToPi(q_4_solns);

        for i_45 = 1:length(q_4_solns)
            q_5 =  q_5_solns(i_45);
            q_4 =  q_4_solns(i_45);
            R_4 = R_6 * rot(-kin.H(:,5), q_5) * rot(-kin.H(:,4), q_4);
            [q_3_solns, q_2_solns] = subproblem2(R_4*-p_02_0, -p_2, -R_4*kin.H(:,3), -R_4*kin.H(:,2));
            q_3_solns = wrapToPi(q_3_solns);
            q_2_solns = wrapToPi(q_2_solns);

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
