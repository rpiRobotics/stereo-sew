function qq = anthro_inv_kin(R, T, SEW_type, psi, GC, kin)
% kin
%   p_W_EE: vector from wrist to EE in zero pose
%   S: position of shoulder
%   d_S_E: distance from shoulder to elbow
%   d_E_W: distance from elbow to wrist

% GC
%   [GC_2 GC_4 GC_6]
%   q_i has the sign of GC_i
%   GC_i = 1 if q_i = 0

% SEW_type
% 'Conventional' or 'Stereographic'

qq = NaN([7,1]);




% Find wrist position
    W = T - R * kin.p_W_EE;

% Find shoulder position
    S = kin.S;

% Find elbow position
    % Use lengths of links to find point C
    % delta is the angle between SW and SE
    d_S_W = norm(W - S);
    cos_delta = (kin.d_S_E^2 + d_S_W^2 - kin.d_E_W^2) / (2 * kin.d_S_E * d_S_W);

    C = S + (W - S) * kin.d_S_E * cos_delta / d_S_W;

    % Find x_c and y_c
    W_hat = (W - S) / d_S_W;

    if isstruct(SEW_type)
        if SEW_type.type == "Conventional"
            y_c = cross(W_hat, SEW_type.V);
            y_c = y_c / norm(y_c);
    
            x_c = cross(y_c, W_hat);
        elseif SEW_type.type == "Stereographic"
            n_hat_ref = vec_normalize(cross(W_hat - SEW_type.R, SEW_type.V));
            x_c = vec_normalize(cross(n_hat_ref, W_hat));
            y_c = cross(W_hat, x_c);
        else
            error("Wrong SEW_type")
        end
    elseif SEW_type == "Conventional"
        V = [1;0;0]; % TODO

        y_c = cross(W_hat, V);
        y_c = y_c / norm(y_c);
    
        x_c = cross(y_c, W_hat);
    
    elseif SEW_type == "Stereographic"
        V_sew = [1;0;0];
        R_sew = [0;0;-1];
        
        n_hat_ref = vec_normalize(cross(W_hat - R_sew, V_sew));
        x_c = vec_normalize(cross(n_hat_ref, W_hat));
        y_c = cross(W_hat, x_c);
    else
        error("Wrong SEW_type")
    end

    % Find elbow position based on (x_c, y_c) and SEW angle
    d_C_E = kin.d_S_E * sqrt(1-cos_delta^2);
    E = C + d_C_E * (x_c * cos(psi) + y_c * sin(psi));

% Find q_1 and q_2 using subproblem 2
    h_1 = [0;0;1]; % Joint 1
    h_2 = [0;1;0]; % Joint 2
    p_S_E_0 = [0;0;1] * kin.d_S_E;
    p_S_E = E-S;

    [q1, q2] = subproblem2(p_S_E_0, p_S_E, h_1, h_2);

    if (q2(1) >= 0) == (GC(1) == 1)
        qq([1 2]) = [q1(1) q2(1)];
    else
        qq([1 2]) = [q1(2) q2(2)];
    end  

% Find q_3 and q_4 using subproblem 2
    
    h_3 = [0;0;1];
    h_4 = [0;-1;0];
    p_E_W_0 = [0;0;1] * kin.d_E_W;
    p_E_W = W - E;
    
    R_2 = rot(h_1, qq(1)) * rot(h_2, qq(2));

    [q3, q4] = subproblem2(p_E_W_0, R_2'*p_E_W, h_3, h_4);

    if (q4(1) >= 0) == (GC(2) == 1)
        qq([3 4]) = [q3(1) q4(1)];
    else
        qq([3 4]) = [q3(2) q4(2)];
    end

% Find q_5 and q_6 using subproblem 2
    % Calculate R at joint 4
    h_5 = [0;0;1];
    h_6 = [0;1;0];
    p_W_EE = T - W;

    R_4 = R_2*rot(h_3, qq(3)) * rot(h_4, qq(4));
    [q5, q6] = subproblem2(kin.p_W_EE, R_4'*p_W_EE, h_5, h_6);

    if (q6(1) >= 0) == (GC(3) == 1)
        qq([5 6]) = [q5(1) q6(1)];
    else
        qq([5 6]) = [q5(2) q6(2)];
    end

% Find q_7
    h_7 = [0;0;1]; % TODO
    R_6 = R_4 * rot(h_5, qq(5)) * rot(h_6, qq(6));
    axang = rotm2axang(R_6' * R);
    qq(7) = axang(4);
    qq(7) = axang(4) * axang(1:3)*h_7;

end


function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
