%% Generate trajectory
kin = robot_kin.kuka;

% Define 4 waypoints in a square
L_wp = 0.4;

% (x,z) changes, (y,R,psi) are constant
wp_xz = [-1 1  1 -1
          1 1 -1 -1]*L_wp;
wp_y = 1e-2 * [1 1 1 1];
% wp_y = 0 * [1 1 1 1];

wp_xyz = [wp_xz(1,:); wp_y; wp_xz(2,:)] + kin.P(:,1);


N = 3000;
[path_xyz,~,~,tt,~] = trapveltraj(wp_xyz,N);

R = eye(3);
psi = pi/4;
GC = [1 1 1];

path_q_conv = NaN([7 N]);
path_q_stereo = NaN([7 N]);

SEW_conv = sew_conv([0;0;1]);
SEW_stereo = sew_stereo([0;0;-1], [0;1;0]);

for i = 1:N
    Q_conv = SEW_IK.IK_2R_2R_3R(R, path_xyz(:,i), SEW_conv, psi, kin);
    path_q_conv(:,i) = pick_soln_given_GC(Q_conv, GC);

    Q_stereo = SEW_IK.IK_2R_2R_3R(R, path_xyz(:,i), SEW_stereo, psi, kin);
    path_q_stereo(:,i) = pick_soln_given_GC(Q_stereo, GC);
end

figure(1)
plot(path_xyz'); title("Trajectory");
ylabel("position (m)"); xlabel("Timestep")

figure(2)
plot(unwrap(path_q_conv')); title("Conventional")
ylabel("q (rad)"); xlabel("Timestep")

figure(3)
plot(unwrap(path_q_stereo')); title("Stereographic")
ylabel("q (rad)"); xlabel("Timestep")

% figure
% show(sim.robot, path_q_conv(:,1)); hold on
% show(sim.robot, path_q_conv(:,150));
% show(sim.robot, path_q_conv(:,end));
% plot3(path_xyz(1,:),path_xyz(2,:),path_xyz(3,:)); hold off

function q = pick_soln_given_GC(Q, GC)
    idx = sign(Q(2,:)) == GC(1) & sign(Q(4,:)) == GC(2) & sign(Q(6,:)) == GC(3);
    q = Q(:,idx);
end
