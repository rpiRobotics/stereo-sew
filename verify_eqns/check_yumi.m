kin = robot_kin.yumi;

offsets = [];
for i = 1:6
    offsets(i) = intersection(kin.P(:,i+1), kin.H(:,i), kin.H(:,i+1));
end
disp(offsets)

function d_LS = intersection(p, h_1, h_2)
A = [h_1 h_2];
p_LS = A * pinv(A)*p;

d_LS = norm(p - p_LS);
end