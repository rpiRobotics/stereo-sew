function [qq, debug] = anthro_inv_kin_all(R, T, psi, kin)
% kin
%   p_W_EE: vector from wrist to EE in zero pose
%   S: position of shoulder
%   d_S_E: distance from shoulder to elbow
%   d_E_W: distance from elbow to wrist

qq = NaN([8, 7]);

V = [0;0;1];


% Find wrist position
    W = T - R * kin.p_W_EE;
    debug.W = W;

% Find shoulder position
    S = kin.S;

% Find elbow position
    % Use lengths of links to find point C
    % delta is the angle between SW and SE
    d_S_W = norm(W - S);
    cos_delta = (kin.d_S_E^2 + d_S_W^2 - kin.d_E_W^2) / (2 * kin.d_S_E * d_S_W);

    C = S + (W - S) * kin.d_S_E * cos_delta / d_S_W;
    debug.C = C;

    % Find x_c and y_c
    W_hat = (W - S) / d_S_W;

    y_c = cross(W_hat, V);
    y_c = y_c / norm(y_c);

    x_c = cross(y_c, W_hat);

    % Find elbow position based on (x_c, y_c) and SEW angle
    d_C_E = kin.d_S_E * sqrt(1-cos_delta^2);
    E = C + d_C_E * (x_c * cos(psi) + y_c * sin(psi));
    debug.E = E;

% Find q_1 and q_2 using subproblem 2
    h_1 = [0;0;1]; % Joint 1
    h_2 = [0;1;0]; % Joint 2
    p_S_E_0 = [0;0;1] * kin.d_S_E;
    p_S_E = E-S;

    [q1, q2] = subproblem2(p_S_E_0, p_S_E, h_1, h_2);
    debug.q12 = [q1 q2];
    qq([1,2,3,4], 1) = q1(1);
    qq([5,6,7,8], 1) = q1(2);
    qq([1,2,3,4], 2) = q2(1);
    qq([5,6,7,8], 2) = q2(2);

% Find q_3 and q_4 using subproblem 2
    
    h_3 = [0;0;1];
    h_4 = [0;-1;0];
    p_E_W_0 = [0;0;1] * kin.d_E_W;
    p_E_W = W - E;
    
    for i = 1:2
        q_1_i = q1(i);
        q_2_i = q2(i);
        R_2 = rot(h_1, q_1_i) * rot(h_2, q_2_i);

        [q3, q4] = subproblem2(p_E_W_0, R_2'*p_E_W, h_3, h_4);
        debug.q34 = [q3 q4];
        if i == 1
            qq([1,2], 3) = q3(1);
            qq([1,2], 4) = q4(1);
            qq([3,4], 3) = q3(2);
            qq([3,4], 4) = q4(2);
        else
            qq([5,6], 3) = q3(1);
            qq([5,6], 4) = q4(1);
            qq([7,8], 3) = q3(2);
            qq([7,8], 4) = q4(2);
        end
    end

% Find q_5 and q_6 using subproblem 2
    % Calculate R at joint 4
    h_5 = [0;0;1];
    h_6 = [0;1;0];
    p_W_EE = T - W;

    for i = [1, 3, 5, 7]
        q_1_i = qq(i,1);
        q_2_i = qq(i,2);
        q_3_i = qq(i,3);
        q_4_i = qq(i,4);

        R_4 = rot(h_1, q_1_i) * rot(h_2, q_2_i)*rot(h_3, q_3_i) * rot(h_4, q_4_i);
        [q5, q6] = subproblem2(kin.p_W_EE, R_4'*p_W_EE, h_5, h_6);

        qq(i, 5) = q5(1);
        qq(i, 6) = q6(1);
        qq(i+1, 5) = q5(2);
        qq(i+1, 6) = q6(2);
    end


% Find q_7
    for i = 1:8
        R_6 = rot(h_1, qq(i,1)) * rot(h_2, qq(i,2))*rot(h_3, qq(i,3)) * rot(h_4, qq(i,4)) * rot(h_5, qq(i,5)) * rot(h_6, qq(i,6));
        axang = rotm2axang(R_6' * R);
        qq(i,7) = axang(4);
    end

end
