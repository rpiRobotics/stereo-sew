kin = define_sawyer();
SEW = sew_conv([0;0;1]);

kin_E = kin;
kin_E.joint_type = [0 0 0];

kin_W = kin;
kin_W.joint_type = [0 0 0 0 0];

kin_2 = kin;
kin_2.joint_type = [0];

%q = rand([7 1])*2*pi - pi
%q = zeros([7 1])

[R_7,p] = fwdkin(kin, q)
R = R_7*kin.RT

p_S = kin.P(:,1);

[~,p_E] = fwdkin(kin_E, q)
[~,p_W] = fwdkin(kin_W, q)

[~,p_2] = fwdkin(kin_2, q)

psi_conv = SEW.fwd_kin(p_S, p_E, p_W)


% zv = [0;0;0];
% kin_mat = [zv p_S p_E p_W p];
% plot3(kin_mat(1,:), kin_mat(2,:), kin_mat(3,:), '-xk')
% axis square
% xlabel("X")
% ylabel("Y")
% zlabel("Z")

% wrist angle is measured from line from wrist to shoulder
% X axis: shoulder - wrist
% Y axis: direction of SEW half-plane

WA_X = vec_normalize(p_S - p_W);
P_WE = p_E - p_W;
WA_Y = vec_normalize(P_WE - WA_X*WA_X'*P_WE);

WA = atan2(WA_Y'*P_WE, WA_X'*P_WE);

% [alignment, q_solns] = alignment_given_wrist_angle(WA,kin,R,p,psi_conv,SEW)
% 
% q


%WA_guesses = linspace(0, 2*pi, 1000);
WA_guesses = linspace(0.3,1.1, 2000);
max_alignments = NaN(size(WA_guesses));
alignment_mat = NaN([length(WA_guesses) 16]);
for i = 1:length(WA_guesses)
    WA_guess = WA_guesses(i);
    [alignment_vec, ~] = alignment_given_wrist_angle(WA_guess,kin,R,p,psi_conv,SEW);
    if ~isempty(alignment_vec)
        max_alignments(i) = max(alignment_vec);
        alignment_mat(i,1:length(alignment_vec)) = alignment_vec;
    end
end

plot(WA_guesses, max_alignments);
yline(1);
xline(WA);

plot(WA_guesses, alignment_mat)
yline(1);
xline(WA);
xlabel("Wrist Angle (rad)")
ylabel("Alignment")
title("Sawyer h_1 Alignment vs Wrist Angle")
%xlim([0.3 1.1])

%%
%ALIGNMENT_THRESH = 1-0.15e-6
ALIGNMENT_THRESH = 1-5e-3
%ALIGNMENT_THRESH = sorted_alignments(9)
%WA_angles = WA_guesses(max_alignments > ALIGNMENT_THRESH)
[max_align_i, ind_max_align_i] = max(alignment_mat)

WA_angles = [];
alignment_mins = [];
for i = 1:16
    if max_align_i(i) > ALIGNMENT_THRESH
        WA_angles = [WA_angles WA_guesses(ind_max_align_i(i))];
        alignment_mins = [alignment_mins max_align_i(i)];
    end
end
WA_angles
alignment_mins

plot(WA_guesses, alignment_mat)
for ang = WA_angles
    xline(ang);
end

solutions = [];

for i = 1:length(WA_angles)
    WA_i = WA_angles(i)
    [alignment_vec, soln_mat] = alignment_given_wrist_angle(WA_i,kin,R,p,psi_conv,SEW);
    solutions = [solutions soln_mat(:,alignment_vec>alignment_mins(i)-1e-6)];
end

solutions

for q_i = solutions
    [R_7_i,p_i] = fwdkin(kin, q_i);
    [~,p_E_i] = fwdkin(kin_E, q_i);
    [~,p_W_i] = fwdkin(kin_W, q_i);
    
    psi_conv_i = sew.fwd_kin(p_S, p_E_i, p_W_i);
    [norm(R_7_i - R_7) norm(p_i - p) norm(psi_conv_i - psi_conv)]
end

%%
robot = importrobot('sawyer.urdf', DataFormat='column');

for i = 1:width(solutions)

    q_i = solutions(:,i);
    show(robot, [q_i(1); 0; q_i(2:end)], "Frames","off");
    %show(robot, [q_i(1); 0; q_i(2:end)], "Visuals","off");
    if i == 1
        hold on
    end
    
end
hold off
light('Position', [1 0 1], 'Color',  0.75*[1 0.9 0.9])
light('Position', [0 1 1], 'Color',  [0.9 0.9 1])
light('Position', [-1 -1 1], 'Color', 0.5*[0.9 0.9 1])
%%
function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
