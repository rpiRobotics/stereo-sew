classdef robot_kinematic_sim_spacebot < handle
    properties (Access = public)
        % constants
        robot
        kin
        ax
        V_conv
        R_stereo
        V_stereo

        % variable
        q
        GC
        quat
        xyz
        sew_conv
        sew_stereo
    end
    
    methods
        function obj = robot_kinematic_sim_spacebot()
            obj.kin = define_spacebot();
            obj.robot = kin2robot(obj.kin);
            obj.q = obj.robot.homeConfiguration;
            %obj.q = [0; 0; 0; deg2rad(90); 0; 0; 0];
            obj.ax = show(obj.robot, obj.q, ...
                FastUpdate=true, PreservePlot=false);

            obj.V_conv = [1;0;0];
            obj.R_stereo = [0;0;-1];
            obj.V_stereo = [1;0;0];
            
            obj.set_q(obj.q)
            obj.fwd_kin()
            obj.fwd_kin_sew()
        end

        function show_plot(obj)
            show(obj.robot, obj.q, ...
                FastUpdate=true, PreservePlot=false, Parent=obj.ax);
            drawnow limitrate
        end
        
        function set_q(obj,q)
            obj.q = q;
            obj.GC = [-1 -1 -1];
            obj.GC([obj.q(2) obj.q(4) obj.q(6)] >= 0) = 1;
        end

        function fwd_kin(obj)
            tform = getTransform(obj.robot, obj.q,'end_effector');
            obj.quat = tform2quat(tform);
            obj.xyz = tform2trvec(tform)';
        end
        
        function [S, E, W] = get_SEW_pos(obj)
            tf_S = getTransform(obj.robot, obj.q,'body_1');
            tf_E = getTransform(obj.robot, obj.q,'body_4');
            tf_W = getTransform(obj.robot, obj.q,'body_7');
            S = tform2trvec(tf_S)';
            E = tform2trvec(tf_E)';
            W = tform2trvec(tf_W)';
        end

        function fwd_kin_sew(obj)
            [S, E, W] = obj.get_SEW_pos();

            conv = conv_sew(S, E, W, obj.V_conv);
            stereo = stereo_sew(S, E, W, obj.R_stereo, obj.V_stereo);

            obj.sew_conv = conv;
            obj.sew_stereo = stereo;
        end

        function inv_kin(obj, quat, T, SEW_type, psi, GC, ~)
            obj.quat = quat;
            obj.xyz = T;
            qq = closed_form_inv_kin(quat2rotm(quat), T, SEW_type, psi, GC, obj.kin);
            
            obj.set_q(qq);
            obj.fwd_kin_sew();
        end

        function inv_kin_ls(obj, quat, T, SEW_type, psi)
            % Find task delta
            R = quat2rotm(quat)'*quat2rotm(obj.quat);
            axang=rotm2axang(R);

            r_delta_theta = axang(4)*axang(1:3)';

            delta_p = T - obj.xyz;
            
            if SEW_type == "None"
                J = geometricJacobian(obj.robot,obj.q,'end_effector');
                delta_q =   pinv(J)*[r_delta_theta; delta_p];
            else
                % Find SEW angle delta
                if SEW_type == "Conventional"
                    delta_psi = psi - obj.sew_conv;
                else
                    delta_psi = psi - obj.sew_stereo;
                end

                J_A = obj.augmented_jacobian(SEW_type);
                delta_q =   pinv(J_A)*[r_delta_theta; delta_p; delta_psi];
            end

            obj.set_q(obj.q + delta_q);

            obj.fwd_kin()
            obj.fwd_kin_sew()
        end

        function J_A = augmented_jacobian(obj, SEW_type)
            J = geometricJacobian(obj.robot,obj.q,'end_effector');
            
            J_E_full = geometricJacobian(obj.robot,obj.q,'body_4');
            J_E_pos = J_E_full(4:6, :);
            
            J_W_full = geometricJacobian(obj.robot,obj.q,'body_7');
            J_W_pos = J_W_full(4:6, :);

            [S, E, W] = obj.get_SEW_pos();

            if SEW_type == "Conventional"
                [J_psi_e, J_psi_w] = conv_sew_jacobian(S, E, W, obj.V_conv);
            else
                [J_psi_e, J_psi_w] = stereo_sew_jacobian(S, E, W, obj.R_stereo, obj.V_stereo);
            end

            J_psi = J_psi_e * J_E_pos + J_psi_w * J_W_pos;
            J_A = [J; J_psi];        
        end

    end
end

