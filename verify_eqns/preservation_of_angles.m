e_SW = rand_normal_vec;
e_r_1 = rand_normal_vec;
e_t = rand_perp_normal_vec(e_r_1);

alpha = rand_angle
e_r_2 = rot(e_t, alpha) * e_r_1;


e_x_1 = e_x(e_SW, e_t, e_r_1);
e_x_2 = e_x(e_SW, e_t, e_r_2);
[beta, is_LS] = subproblem.sp_1(e_x_1, e_x_2, -e_SW)

function e_x = e_x(e_SW, e_t, e_r)
    k_r = cross(e_SW - e_t, e_r);
    k_x = cross(k_r, e_SW);
    e_x = k_x / norm(k_x);
end