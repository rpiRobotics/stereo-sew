sawyer = sawyer_ros()

%%
q = zeros([7 1])

for i = 1:100
    q = q+deg2rad(1);
    sawyer.send_position_cmd(q)
    pause(0.1)
end

%%
qq =wrapToPi(path_q_stereo)

for i = 1:length(qq)
    sawyer.send_position_cmd(qq(:,i))
    pause(1/20)
end
