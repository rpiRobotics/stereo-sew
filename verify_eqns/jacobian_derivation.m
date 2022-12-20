p_SW = rand_vec;
e_SW = p_SW / norm(p_SW);
p_SE = rand_vec;

% e_r = rand_normal_vec;
% e_t = rand_perp_normal_vec(e_r);
e_r = rand_vec; % General!
e_t = rand_vec; % General!

p_CE = (eye(3) - e_SW *e_SW') * p_SE;
e_CE = p_CE / norm(p_CE);

k_r = cross(e_SW - e_t, e_r);
k_x = cross(k_r, e_SW);
e_x = k_x / norm(k_x);
e_y = cross(e_SW, e_x);

W_dot = rand_vec;
E_dot = rand_vec;
e_SW_dot = -hat(e_SW)^2 /norm(p_SW) * W_dot;
k_r_dot = cross(-e_r, e_SW_dot);

p_CE_dot = (eye(3) - e_SW *e_SW') * E_dot - (e_SW_dot*e_SW'*p_SE + e_SW * e_SW_dot'*p_SE);

DELTA = 1e-12;
%% First term
T1 = 1/norm(p_CE) * cross(e_SW,e_CE)' * p_CE_dot
T2 = 1/norm(p_CE) * cross(e_SW,e_CE)' * (E_dot - dot(e_SW,p_SE)/norm(p_SW)*W_dot)

assert(abs(T1-T2) < DELTA)

%% Second term

T1 = -cross(e_y,k_r)' / norm(k_x) * W_dot
T2 = -dot(e_SW,k_r) / norm(k_x) * e_x' * W_dot
T3 = dot(e_SW,cross(e_t,e_r)) / norm(k_x) * e_x' * W_dot

assert(abs(T1-T2) < DELTA)
assert(abs(T1-T3) < DELTA)

%% Third term

T1 = cross(e_y,p_SW)' / norm(k_x)*k_r_dot
T2 = norm(p_SW)/norm(k_x) * e_x' * (hat(e_r) * hat(e_SW)^2 / norm(p_SW)) * W_dot
T3 = dot(e_SW,e_r) / norm(k_x) * e_y' * W_dot

assert(abs(T1-T2) < DELTA)
assert(abs(T1-T3) < DELTA)