function kin = kuka
    % ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = zeros(3,1);

    kin.P = [(0.1575+0.2025)*ez zv (0.2045+0.2155)*ez zv (0.1845+0.2155)*ez zv zv (0.0810+0.0450)*ez];
    kin.H=[ez ey ez -ey ez ey ez];
    kin.joint_type=[0 0 0 0 0 0 0];
end