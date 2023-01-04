function kin = SSRMS
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = zeros(3,1);

    S = [NaN 635 504 504 504 635] / 1e3; % In joint direction
    a = [0   380 6850 6850 380 0] / 1e3; % In offset direction

    kin.P = [zv zv S(2)*ey+a(2)*ex S(3)*ez+a(3)*ex  S(4)*ez+a(4)*ex S(5)*ez+a(5)*ey zv S(6)*ex];

    kin.H=[ex ey ez ez ez ey ex];
    kin.joint_type=[0 0 0 0 0 0 0];
end