function kin = sawyer
    a1=.081;d2=.1925;d3=.4;d4=.1685;d5=.4;d6=.1363;d7=.1345;
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = zeros(3,1);

    kin.P = [0.317*ez a1*ex+d2*ey zv d3*ex -d4*ey d5*ex+d6*ey zv d7*ex];
    %kin.RT = eul2rotm(deg2rad([-90 -10 -90]));
    kin.H=[ez ey ex ey ex ey ex];
    kin.joint_type=[0 0 0 0 0 0 0];
end