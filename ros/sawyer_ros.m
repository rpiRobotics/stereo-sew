classdef sawyer_ros < handle
    properties
        joint_pub
        cmd_msg
    end
    
    methods
        function obj = sawyer_ros()
            local_file_path = mfilename( 'fullpath' );
            local_folder_path = fileparts( local_file_path );
            addpath(fullfile(local_folder_path, "matlab_msg_gen_ros1/win64/src/intera_core_msgs/m/"));
            [status,ip_address] = system("wsl hostname -I");
            if status~=0
                error("Couldn't get WSL IP address!")
            end
            ip_address = strtrim(ip_address);
            try
                rosinit(ip_address);
            catch ME
                fprintf(2,'%s\n',ME.message);
            end

            obj.init_cmd_msg()
        end
        

        function init_cmd_msg(obj)
            obj.joint_pub = rospublisher('/robot/limb/right/joint_command','intera_core_msgs/JointCommand');
    
            obj.cmd_msg = rosmessage(obj.joint_pub);
            obj.cmd_msg.Mode = obj.cmd_msg.POSITIONMODE;
            obj.cmd_msg.Names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
            obj.cmd_msg.Position = zeros([1,7]); 
            obj.cmd_msg.Header.Seq = obj.cmd_msg.Header.Seq + 1;
            obj.cmd_msg.Header.Stamp = rostime('now');
        end

        function send_position_cmd(obj, q)
            obj.cmd_msg.Position = q;
            obj.cmd_msg.Header.Stamp = obj.cmd_msg.Header.Stamp + 1;
            obj.cmd_msg.Header.Stamp = rostime('now');
            send(obj.joint_pub,obj.cmd_msg);
        end
    end
end

