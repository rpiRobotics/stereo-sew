function [qq, stop_condition, err_hist] = iter_IK(R, T, psi, q_guess, kin, SEW_type, V_conv, R_stereo, V_stereo, S)

MAX_ITER = 500;
MAX_ERR = 1e-3;


%LEARNING_RATE = 2e-3;
LEARNING_RATE = 0.2;
WEIGHTS =[50;50;50;1;1;1; 50]; 

q_i = q_guess;

err_hist = NaN([7 MAX_ITER]);

for i = 1:MAX_ITER
    % Fwd Kin and find error
    [R_q, T_q, E, W, J_T, J_E_pos, J_W_pos] = fwd_kin_diff_with_SEW(kin, q_i);

    if SEW_type == "Conventional"
        [J_psi_e, J_psi_w] = conv_sew_jacobian(S, E, W, V_conv);
    else
        [J_psi_e, J_psi_w] = stereo_sew_jacobian(S, E, W, R_stereo, V_stereo);
    end

    J_psi = J_psi_e * J_E_pos + J_psi_w * J_W_pos;
    J = [J_T; J_psi];

   
    R_err = R_q * R';
    axang = rotm2axang(R_err);
    err_angle = axang(4) * axang(1:3);

    err_xyz = T_q - T;
    
    if SEW_type == "Conventional"
        psi_q = conv_sew(S, E, W, V_conv);
    else
        psi_q = stereo_sew(S, E, W, R_stereo, V_stereo);
    end

    err_psi = wrapToPi(psi_q - psi);

    err_vec = [err_angle'; err_xyz ; err_psi];

    err_hist(:,i) = err_vec;

    if norm(err_vec) < MAX_ERR
        stop_condition = 'Error below MAX_ERR';
        qq = q_i;
        return
    end

    % Update

    %q_i = q_i - LEARNING_RATE * pinv(J)*err_vec;
    
    q_i = q_i - LEARNING_RATE * inv(J'*J + 0.01*diag(1./WEIGHTS))*J'*err_vec;  
%     if i < MAX_ITER/2
%         q_i = q_i + rand(size(q_i))*0.01;
%     end
    

    %q_i = q_i - LEARNING_RATE * J'*inv(J*J' + 0.01*diag(1./WEIGHTS))*err_vec;
    %q_i = q_i - LEARNING_RATE *J'*err_vec;
end

stop_condition = 'Iterations passed MAX_ITER';
qq = q_i;


end
