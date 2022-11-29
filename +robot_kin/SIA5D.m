function kin = SIA5D
    % https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/sia-series/sia5d

    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = zeros(3,1);

    kin.P = [0.310*ez zv zv 0.270*ez+0.85*ex 0.060*ez+0.270*ex zv zv 0.145*ex];
    kin.H=[ez -ey ez -ey ex -ey ex];
    kin.joint_type=[0 0 0 0 0 0 0];
end