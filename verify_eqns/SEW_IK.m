p_SW = rand_vec;
p_SE = rand_vec;
e_SW = p_SW / norm(p_SW);

k_SEW = cross(p_SW, p_SE);
n_SEW = k_SEW / norm(k_SEW);

p_CE = -hat(e_SW)^2 * p_SE;
e_CE = p_CE / norm(p_CE);
%% General SEW angle, random e_x
e_x = rand_perp_normal_vec(e_SW);
%%
e_y = cross(e_SW, e_x);

[subproblem.sp_1(e_x, p_SE, e_SW)
subproblem.sp_1(e_x, e_CE, e_SW)
subproblem.sp_1(e_y, n_SEW, e_SW)
atan2(e_y'*p_SE, e_x'*p_SE)
atan2(e_y'*p_CE, e_x'*p_CE)
atan2(-e_x'*n_SEW, e_y'*n_SEW)
atan2(-e_x'*k_SEW, e_y'*k_SEW)]

%% Conventional SEW angle
e_r = rand_normal_vec;
k_y = cross(p_SW, e_r);
e_y = k_y / norm(k_y);
e_x = cross(e_y, e_SW);

[subproblem.sp_1(e_r, p_SE, e_SW)
atan2(cross(e_SW, e_r)'*p_SE, -(hat(e_SW)^2 * e_r)'*p_SE)
atan2(e_SW'*cross(e_r, p_CE), e_r'*p_CE)

atan2(cross(e_SW,k_y)'*k_SEW, -(hat(e_SW)^2*k_y)'*k_SEW)
atan2(e_SW'*cross(k_y,k_SEW),k_y'*k_SEW)]

%% Stereographic SEW angle
e_r = rand_vec; % General!
e_t = rand_vec; % General!

k_r = cross(e_SW - e_t, e_r);
k_x = cross(k_r,p_SW);
e_x = k_x / norm(k_x);

[subproblem.sp_1(k_r, k_SEW, e_SW)
atan2(e_SW'*cross(k_r,k_SEW), k_SEW'*k_r)
atan2(k_r'*p_CE, -e_SW'*cross(k_r,p_CE))]