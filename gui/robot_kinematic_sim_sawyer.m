classdef robot_kinematic_sim_sawyer < handle
    properties (Access = public)
        % constants
        robot
        kin
        kin_approx
        ax
        V_conv
        R_stereo
        V_stereo
        body_names
        p_S

        % variable
        q
        GC
        quat
        xyz
        sew_conv
        sew_stereo
    end
    
    methods
        function obj = robot_kinematic_sim_sawyer()
            obj.robot = importrobot('sawyer.urdf', DataFormat='column');
            
            

            bodyElbow = rigidBody('elbow');
            tform = trvec2tform([0;0;0.042]');
            setFixedTransform(bodyElbow.Joint,tform);
            addBody(obj.robot,bodyElbow,'right_l3');

            ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];
            zz=[0;0;0];
            
            obj.p_S = 0.317*ez;

            a1=.081;d2=.1925;d3=.4;d4=.1685;d5=.4;d6=.1363;d7=.1345;
            
            obj.kin.P = [0.317*ez a1*ex+d2*ey zz d3*ex -d4*ey d5*ex+d6*ey zz d7*ex];
            obj.kin.RT = eul2rotm(deg2rad([-90 -10 -90]));
            obj.kin.H=[ez ey ex ey ex ey ex];
            obj.kin.joint_type=[0 0 0 0 0 0 0];

            obj.kin.q_bound_lower = constraintJointBounds(obj.robot).Bounds([1 3:8],1);
            obj.kin.q_bound_upper = constraintJointBounds(obj.robot).Bounds([1 3:8],2);


            obj.kin_approx = obj.kin;
            obj.kin_approx.P = [0.317*ez zz a1*ex+d2*ey+d3*ex zz -d4*ey+d5*ex+d6*ey zz zz d7*ex];


            q_with_screen = obj.robot.homeConfiguration;
            obj.q = q_with_screen([1 3:8]);
            obj.ax = show(obj.robot, [obj.q(1); 0; obj.q(2:end)], ...
                FastUpdate=true, PreservePlot=false);
            light('Position', [1 0 1], 'Color',  0.75*[1 0.9 0.9])
            light('Position', [0 1 1], 'Color',  [0.9 0.9 1])
            light('Position', [-1 -1 1], 'Color', 0.5*[0.9 0.9 1])

            obj.body_names.EE = 'right_hand';
            obj.body_names.S = 'base';
            %obj.body_names.E = 'right_l3';
            obj.body_names.E = 'elbow';
            obj.body_names.W = 'right_wrist';


            obj.V_conv = [1;0;0];
            obj.R_stereo = [0;0;-1];
            obj.V_stereo = [1;0;0];

            obj.set_q(obj.q);
            obj.fwd_kin();
            obj.fwd_kin_sew();
        end
        
        function show_plot(obj)
            show(obj.robot, [obj.q(1); 0; obj.q(2:end)], ...
                FastUpdate=true, PreservePlot=false, Parent=obj.ax);
            drawnow limitrate
        end
        
        function set_q(obj,q)
            obj.q = q;
            obj.GC = [-1 -1 -1];
            obj.GC([obj.q(2) obj.q(4) obj.q(6)] >= 0) = 1;
        end

        function fwd_kin(obj)
            tform = getTransform(obj.robot, ...
                [obj.q(1); 0; obj.q(2:end)],'right_hand');
            obj.quat = tform2quat(tform);
            obj.xyz = tform2trvec(tform)';
        end

        function [S, E, W] = get_SEW_pos(obj)
            tf_E = getTransform(obj.robot, [obj.q(1); 0; obj.q(2:end)],obj.body_names.E);
            tf_W = getTransform(obj.robot, [obj.q(1); 0; obj.q(2:end)],'right_wrist');
            S = obj.p_S;
            E = tform2trvec(tf_E)';
            W = tform2trvec(tf_W)';
        end

        function [S, E, W] = get_SEW_pos_by_q(obj, q)
            tf_E = getTransform(obj.robot, [q(1); 0; q(2:end)],obj.body_names.E);
            tf_W = getTransform(obj.robot, [q(1); 0; q(2:end)],'right_wrist');
            S = obj.p_S;
            E = tform2trvec(tf_E)';
            W = tform2trvec(tf_W)';
        end

        function [conv, stereo] = fwd_kin_sew(obj)
            [S, E, W] = obj.get_SEW_pos();

            conv = conv_sew(S, E, W, obj.V_conv);
            stereo = stereo_sew(S, E, W, obj.R_stereo, obj.V_stereo);

            obj.sew_conv = conv;
            obj.sew_stereo = stereo;
        end

        function [success, fail_reason] = inv_kin(obj, quat, T, SEW_type, psi, GC, use_q_as_guess, use_joint_limits)
            % TODO use options
            R = quat2rotm(quat);
            if use_q_as_guess
                qq_guess = obj.q;
            else
                qq_guess = closed_form_inv_kin(R, T, SEW_type, psi, GC, obj.kin_approx);
            end

            

            % Use iteration to update q so (R, T, phi) is satisfied
            
            if isstruct(SEW_type)
                if SEW_type.type == "Conventional"
                    [qq, stop_condition, ~] = iter_IK(R, T, psi, qq_guess, obj.kin, SEW_type.type, SEW_type.V, [], [], obj.p_S);
                elseif SEW.type == "Stereographic"
                    [qq, stop_condition, ~] = iter_IK(R, T, psi, qq_guess, obj.kin, SEW_type.type, [], SEW_type.R, SEW_type.V, obj.p_S);
                end
            else
            [qq, stop_condition, ~] = iter_IK(R, T, psi, qq_guess, obj.kin, SEW_type, obj.V_conv, obj.R_stereo, obj.V_stereo, obj.p_S);
            end

            if stop_condition == "Iterations passed MAX_ITER"            
                disp(stop_condition);
                success = false;
                fail_reason = "singularity";
                return
            end

            if use_joint_limits
                lb = all(qq > obj.kin.q_bound_lower);
                ub = all(qq < obj.kin.q_bound_upper);
                if ~(lb && ub)
                    disp("Past joint limits");
                    success = false;
                    fail_reason = "joint_limit";
                    return
                end
            end
            
            % Double check GC is satisfied
            obj.set_q(qq);

            if ~all(obj.GC == GC)
                disp('GC conditions failed!')
            end

            obj.fwd_kin();
            obj.fwd_kin_sew();
            
            fail_reason = nan;
            success = true;
        end

        function inv_kin_ls(obj, quat, T, SEW_type, psi)
            % Find task delta
            R = quat2rotm(quat)'*quat2rotm(obj.quat);
            axang=rotm2axang(R);

            r_delta_theta = axang(4)*axang(1:3)';

            delta_p = T - obj.xyz;
            
            if SEW_type == "None"
                J = geometricJacobian(obj.robot, [obj.q(1); 0; obj.q(2:end)],'right_hand');
                J = J(:,[1,3:8]);
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

            obj.fwd_kin();
            obj.fwd_kin_sew();
        end

        function J_A = augmented_jacobian_by_q(obj, SEW_type, q)
            J = geometricJacobian(obj.robot, [q(1); 0; q(2:end)],'right_hand');
            J = J(:,[1,3:8]);
            
            J_E_full = geometricJacobian(obj.robot, [q(1); 0; q(2:end)],obj.body_names.E);
            J_E_full = J_E_full(:,[1,3:8]);
            J_E_pos = J_E_full(4:6, :);
            
            J_W_full = geometricJacobian(obj.robot, [q(1); 0; q(2:end)],'right_wrist');
            J_W_full = J_W_full(:,[1,3:8]);
            J_W_pos = J_W_full(4:6, :);

            [S, E, W] = obj.get_SEW_pos_by_q(q);

            if SEW_type == "Conventional"
                [J_psi_e, J_psi_w] = conv_sew_jacobian(S, E, W, obj.V_conv);
            else
                [J_psi_e, J_psi_w] = stereo_sew_jacobian(S, E, W, obj.R_stereo, obj.V_stereo);
            end

            J_psi = J_psi_e * J_E_pos + J_psi_w * J_W_pos;
            J_A = [J; J_psi];      
        end

        function J_A = augmented_jacobian(obj, SEW_type)
              J_A = augmented_jacobian_by_q(obj, SEW_type, obj.q);
        end

        
    end
end

