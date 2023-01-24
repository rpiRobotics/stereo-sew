classdef IK_2R_3Rp_2R
    methods (Static)
        function [P, S] = setup()
            zv = [0;0;0];

            S.Q = rand_angle([7 1]);

            P.sew = sew_conv(rand_normal_vec);

            P.kin.joint_type = zeros(1,7);
            P.kin.P = [rand_vec zv rand_vec rand_vec rand_vec rand_vec zv rand_vec];
            P.kin.H = rand_normal_vec(7);
            P.kin.H(:,4) = P.kin.H(:,3);
            P.kin.H(:,5) = P.kin.H(:,3);

            h_3_0 = rot(P.kin.H(:,1), S.Q(1)) * rot(P.kin.H(:,2), S.Q(2)) * P.kin.H(:,3);
            [P.R, P.T, P_SW] = fwdkin_inter(P.kin, S.Q, [1 6]);
            P.psi = P.sew.fwd_kin(P_SW(:,1),P_SW(:,1) + h_3_0, P_SW(:,2));
        end

%         function P = setup_LS()
%             zv = [0;0;0];
%             
%             P.sew = sew_conv(rand_normal_vec);
% 
%             P.kin.joint_type = zeros(1,7);
%             P.kin.P = [rand_vec zv rand_vec rand_vec rand_vec rand_vec zv zv]; % Task frame at wrist
%             P.kin.H = rand_normal_vec(7);
%             
%             % Perpendicular shoulder joints
%             P.kin.H(:,2) = rand_perp_normal_vec(P.kin.H(:,1));
%             P.kin.H(:,3) = rand_perp_normal_vec(P.kin.H(:,2));
%             P.kin.H(:,4) = P.kin.H(:,3);
%             P.kin.H(:,5) = P.kin.H(:,3);
%             P.kin.H(:,5) = rand_perp_normal_vec(P.kin.H(:,5));
%             P.kin.H(:,6) = rand_perp_normal_vec(P.kin.H(:,6));
% 
%             P.R = rot(rand_normal_vec, rand_angle);
%             P.T = 10*rand_vec;
%             P.psi = rand_angle;
%         end

        function S = run(P)
            [S.Q, S.is_LS] = SEW_IK.IK_2R_3Rp_2R(P.R, P.T, P.sew, P.psi, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = SEW_IK.IK_2R_3Rp_2R_mex(P.R, P.T, P.sew, P.psi, P.kin);
        end

        function generate_mex()
            P = SEW_IK_setups.IK_2R_3Rp_2R.setup(); %#ok<NASGU> 
            codegen -report +SEW_IK/IK_2R_3Rp_2R.m -args {P.R, P.T, P.sew, P.psi, P.kin}
        end

        function [e, e_R, e_T, e_psi] = error(P, S)
            e_R = NaN([1 width(S.Q)]);
            e_T = NaN([1 width(S.Q)]);
            e_psi = NaN([1 width(S.Q)]);
            
            for i = 1:width(S.Q)
                [R_t, T_t, P_SW_t] = fwdkin_inter(P.kin, S.Q(:,i), [1 6]);
                e_R(i) = norm(R_t - P.R);
                e_T(i) = norm(T_t - P.T);
                
                h_3_0_t = rot(P.kin.H(:,1), S.Q(1,i)) * rot(P.kin.H(:,2), S.Q(2,i)) * P.kin.H(:,3);
                psi_t = P.sew.fwd_kin(P_SW_t(:,1),P_SW_t(:,1) + h_3_0_t,P_SW_t(:,2));
                e_psi(i) = norm(psi_t - P.psi);
            end
            e = e_R + e_T + e_psi;
        end
    end
end