addpath("./matlab_msg_gen_ros1/win64/src/intera_core_msgs/m/")

%%
[status,ip_address] = system("wsl hostname -I")
ip_address = strtrim(ip_address)
rosinit(ip_address)

%%
sub = rossubscriber('/robot/joint_states','DataFormat','struct');

[feedback_msg,status,statustext] = receive(sub,1)

%enable_pub = rospublisher('/robot/set_super_enable',msgtype)


%%
joint_pub = rospublisher('/robot/limb/right/joint_command','intera_core_msgs/JointCommand')

msg = rosmessage(joint_pub)
msg.Mode = msg.POSITIONMODE
%msg.Names = ["right_j6", "right_j5", "right_j4", "right_j3", "right_j2", "right_j1", "right_j0"];
msg.Names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
%msg.Position = [0 0 0 0 1 0 0 0]*deg2rad(10)
msg.Position = [0.44306910945301803, 1.817598298595035, 2.7958235468001615, -1.589300730783936, 0.010672899352333655, -3.1244247205300306, 2.3654556749206463];
%msg.Effort = [0.00047069237088154524, 0.5218596169707969, -0.7832788633149375, 3.247392341460261, -1.6989969232725575, -22.365931732553268, -0.015812619852810732]

msg.Header.Seq = msg.Header.Seq + 1;
msg.Header.Stamp = rostime('now');

%%
for i = 1:100
msg.Position = msg.Position + deg2rad(1)
msg.Header.Stamp = msg.Header.Stamp + 1;
msg.Header.Stamp = rostime('now');
%msg.Effort = msg.Effort*1.01
send(joint_pub,msg);
pause(0.1)
end

%%
rosshutdown
