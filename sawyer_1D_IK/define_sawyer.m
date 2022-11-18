function kin = define_sawyer()
ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];
zz=[0;0;0];

a1=.081;d2=.1925;d3=.4;d4=.1685;d5=.4;d6=.1363;d7=.1345;

% O_2, O_3 coincident
% O_4, O_5 coincident (Elbow)
% O_6, O_7 conincident (Wrist)
kin.P = [0.317*ez a1*ex+d2*ey zz d3*ex-d4*ey zz d5*ex+d6*ey zz d7*ex];

kin.RT = eul2rotm(deg2rad([-90 -10 -90]));
kin.H=[ez ey ex ey ex ey ex];
kin.joint_type=[0 0 0 0 0 0 0];
end
