classdef sew_stereo
    properties
        R
        V
    end
    
    methods
        function obj = sew_stereo(R, V)
            obj.R = R;
            obj.V = V;
        end
        
        function psi = fwd_kin(obj, S, E, W)
            E = E - S;
            W = W - S;
            w_hat = vec_normalize(W);
            
            n_hat_sew = vec_normalize(cross(W,E));

            n_hat_ref = vec_normalize(cross(w_hat - obj.R, obj.V));
            
            psi = atan2( ...
                    n_hat_sew'*cross(w_hat, n_hat_ref), ...
                    n_hat_sew'*n_hat_ref);

        end

        function psi = alt_fwd_kin(obj, S, E, W)
            E = E - S;
            W = W - S;
            w_hat = vec_normalize(W);
            
            n_sew = cross(W,E);
            n_phi = -cross(w_hat - obj.R, n_sew);
            
            psi = atan2( ...
                    n_phi'*cross(-obj.R, obj.V), ...
                    n_phi'*obj.V);
        end

        function [e_CE, n_SEW] = inv_kin(obj, S, W, psi)
            p_SW = W-S;
            e_SW = vec_normalize(p_SW);
            k_r = cross(e_SW - obj.R, obj.V);
            k_x = cross(k_r, p_SW);
            e_x = vec_normalize(k_x);
            
            e_CE = rot(e_SW, psi)*e_x;
            n_SEW = cross(e_SW, e_CE);
        end

        function [J_e, J_w] = jacobian(obj, S, E, W)
            E = E - S;
            W = W - S;
            w_hat = vec_normalize(W);

            n_ref = cross(w_hat - obj.R, obj.V);

            x_c = cross(n_ref, W);
            x_hat_c = vec_normalize(x_c);
            y_hat_c = cross(w_hat, x_hat_c);
            
            p = (eye(3) - w_hat*w_hat')*E;
            p_hat = vec_normalize(p);
            
            J_e = cross(w_hat, p_hat)' / norm(p);
            
            J_w_1 = (w_hat'*obj.V) /norm(x_c) * y_hat_c';
            J_w_2 = (w_hat'*cross(obj.R,obj.V))/norm(x_c) * x_hat_c';
            J_w_3 = (w_hat'*E)/norm(W)/norm(p) * cross(w_hat, p_hat)';
            
            J_w = J_w_1 + J_w_2 - J_w_3;

        end
    end
end

function n = vec_normalize(vec)
    n =  vec / norm(vec);
end