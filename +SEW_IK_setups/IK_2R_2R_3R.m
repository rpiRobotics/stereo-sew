classdef IK_2R_2R_3R
    methods (Static)
        function [P, S] = setup()
            zv = [0;0;0];

            S.Q = rand_angle([7 1]);

            P.sew = sew_conv(rand_normal_vec);

            P.kin.joint_type = zeros(1,7);
            P.kin.P = [rand_vec zv rand_vec zv rand_vec zv zv rand_vec];
            P.kin.H = rand_normal_vec(7);

            [P.R, P.T, P_SEW] = fwdkin_inter(P.kin, S.Q, [1 3 5]);
            P.psi = P.sew.fwd_kin(P_SEW(:,1),P_SEW(:,2),P_SEW(:,3));
        end

        function P = setup_LS()
            zv = [0;0;0];
            
            P.sew = sew_conv(rand_normal_vec);

            P.kin.joint_type = zeros(1,7);
            P.kin.P = [rand_vec zv rand_vec zv rand_vec zv zv zv]; % Task frame at spherical wrist
            P.kin.H = rand_normal_vec(7);
            
            % 2R joints must be perpendicular to achieve any rotation
            P.kin.H(:,2) = rand_perp_normal_vec(P.kin.H(:,1));
            P.kin.H(:,4) = rand_perp_normal_vec(P.kin.H(:,3));

            % Spherical joint must have perpendicular joints
            P.kin.H(:,6) = rand_perp_normal_vec(P.kin.H(:,5));
            P.kin.H(:,7) = rand_perp_normal_vec(P.kin.H(:,6));

            P.kin.P(:,3) = (eye(3)-P.kin.H(:,2)*P.kin.H(:,2)')*P.kin.P(:,3); % Spherical elbow workspace
            P.kin.P(:,5) = (eye(3)-P.kin.H(:,4)*P.kin.H(:,4)')*P.kin.P(:,5); % Spehrical wrist workspace (around elbow)

            P.R = rot(rand_normal_vec, rand_angle);
            P.T = 10*rand_vec;
            P.psi = rand_angle;
        end

        function S = run(P)
            [S.Q, S.is_LS] = SEW_IK.IK_2R_2R_3R(P.R, P.T, P.sew, P.psi, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = SEW_IK.IK_2R_2R_3R_mex(P.R, P.T, P.sew, P.psi, P.kin);
        end

        function generate_mex()
            P = SEW_IK_setups.IK_2R_2R_3R.setup(); %#ok<NASGU> 
            codegen -report +SEW_IK/IK_2R_2R_3R.m -args {P.R, P.T, P.sew, P.psi, P.kin}
        end

        function [e, e_R, e_T, e_psi] = error(P, S)
            e_R = NaN([1 width(S.Q)]);
            e_T = NaN([1 width(S.Q)]);
            e_psi = NaN([1 width(S.Q)]);
            
            for i = 1:width(S.Q)
                [R_t, T_t, P_SEW_t] = fwdkin_inter(P.kin, S.Q(:,i), [1 3 5]);
                e_R(i) = norm(R_t - P.R);
                e_T(i) = norm(T_t - P.T);
                
                psi_t = P.sew.fwd_kin(P_SEW_t(:,1),P_SEW_t(:,2),P_SEW_t(:,3));
                e_psi(i) = norm(wrapToPi(psi_t - P.psi));
            end
            e = e_R + e_T + e_psi;
        end
    end
end