function kin = define_spacebot()

x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = zeros(3,1);

l1 = 0.48;
l2 = 0.37;
l3 = 1.06;
l4 = 0.28;
l5 = 1.18;
l6 = 0.37;

kin.H = [z0 -y0 z0 -y0 z0 -y0 z0];
kin.P = [l1*z0 zed [0; -l2; l3] zed [0; l4; l5] zed zed l6*z0];
kin.joint_type = zeros(1,7);


kin.limit.lower_joint_limit = [-228;-223;-260;-125;-290;-184;-360]*pi/180;
kin.limit.upper_joint_limit = [128;126;134;113;106;178;150]*pi/180;

end
