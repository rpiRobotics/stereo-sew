classdef IK_3R_R_2R_R
    methods (Static)
        function [P, S] = setup()
            zv = [0;0;0];

            S.Q = rand_angle([7 1]);

            P.sew = sew_conv(rand_normal_vec);

            P.kin.joint_type = zeros(1,7);
            P.kin.P = [rand_vec zv zv rand_vec rand_vec zv rand_vec rand_vec];
            P.kin.H = rand_normal_vec(7);

            [P.R, P.T, P_SEW] = fwdkin_inter(P.kin, S.Q, [1 4 7]);
            P.psi = P.sew.fwd_kin(P_SEW(:,1),P_SEW(:,2),P_SEW(:,3));
        end

        function S = run(P)
            [S.Q, S.is_LS] = SEW_IK.IK_3R_R_2R_R(P.R, P.T, P.sew, P.psi, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = SEW_IK.IK_3R_R_2R_R_mex(P.R, P.T, P.sew, P.psi, P.kin);
        end

        function generate_mex()
            P = SEW_IK_setups.IK_3R_R_2R_R.setup(); %#ok<NASGU> 
            codegen -report +SEW_IK/IK_3R_R_2R_R.m -args {P.R, P.T, P.sew, P.psi, P.kin}
        end

        function [e, e_R, e_T, e_psi] = error(P, S)
            e_R = NaN([1 width(S.Q)]);
            e_T = NaN([1 width(S.Q)]);
            e_psi = NaN([1 width(S.Q)]);
            
            for i = 1:width(S.Q)
                [R_t, T_t, P_SEW_t] = fwdkin_inter(P.kin, S.Q(:,i), [1 4 7]);
                e_R(i) = norm(R_t - P.R);
                e_T(i) = norm(T_t - P.T);
                
                psi_t = P.sew.fwd_kin(P_SEW_t(:,1),P_SEW_t(:,2),P_SEW_t(:,3));
                e_psi(i) = norm(psi_t - P.psi);
            end
            e = e_R + e_T + e_psi;
        end
    end
end