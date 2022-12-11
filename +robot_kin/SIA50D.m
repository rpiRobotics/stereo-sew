function kin = SIA50D
    % https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/sia-series/sia50d

    d1 = 0.540;
    a1 = 0.145;
    d3 = 0.875;
    d5 = 0.610;
    dT = 0.350;

    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = zeros(3,1);

    kin.P = [d1*ez a1*ex d3*ez zv zv d5*ez zv dT*ez];
    kin.H = [ez -ey ez -ey ez -ey ez];
    kin.joint_type = [0 0 0 0 0 0 0];
    
end