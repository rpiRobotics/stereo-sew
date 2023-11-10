classdef sew_conv
    % Conventional SEW angle

    properties
        e_r % Reference unit vector
    end
    
    methods
        function obj = sew_conv(e_r)
            obj.e_r = e_r;
        end
        
        function psi = fwd_kin(obj, S, E, W)
            % Given shoulder, elbow, and wrist postitions,
            % Find SEW angle psi

            p_SE = E - S;
            e_SW = vec_normalize(W - S);

            psi = subproblem.sp_1(obj.e_r, p_SE, e_SW);
        end

        function [e_CE, n_SEW] = inv_kin(obj, S, W, psi)
            % Given shoulder and wrist positions, and given SEW angle psi
            % Find e_CE, unit vector which points towards elbow,
            % and find n_SEW, which is normal to SEW plane

            e_SW = vec_normalize(W - S);
            e_y = vec_normalize(cross(e_SW, obj.e_r));

            n_SEW = rot(e_SW, psi) * e_y;
            e_CE = cross(n_SEW, e_SW);
        end

        function [J_psi_E, J_psi_W] = jacobian(obj, S, E, W)
            % Given shoulder, elbow, and wrist positions,
            % Find Jacobians of SEW angle w.r.t. elbow and wrist velocities
            
            p_SE = E - S;
            p_SW = W - S;
            e_SW = vec_normalize(p_SW);
            k_y = cross(p_SW, obj.e_r);
            e_y = vec_normalize(k_y);
            
            p_CE = -cross(e_SW, cross(e_SW, p_SE));
            e_CE = vec_normalize(p_CE);

            J_psi_E = cross(e_SW, e_CE)' / norm(p_CE);
            
            J_w_1 = dot(e_SW, obj.e_r)/ norm(k_y) * e_y';
            J_w_2 = dot(e_SW, p_SE)/norm(p_SW)/norm(p_CE) * cross(e_SW, e_CE)';
            
            J_psi_W = J_w_1 - J_w_2;
        end
    end
end

function n = vec_normalize(vec)
    n =  vec / norm(vec);
end