e_SW = rand_normal_vec;
e_r = rand_normal_vec;

%% Case 1: ||e_t|| <= 1

e_t = rand_perp_normal_vec(e_r) * rand;

e_SW_test_p = e_t + sqrt(1-norm(e_t)^2) * e_r;
e_SW_test_n = e_t - sqrt(1-norm(e_t)^2) * e_r;

k_x(e_SW_test_p, e_t, e_r)
k_x(e_SW_test_n, e_t, e_r)

%% Case 2: ||e_r|| >= 1

e_t = rand_perp_normal_vec(e_r) * (1 + rand*10);

e_SW_test_p = e_t / norm(e_t)^2 + sqrt(1 - 1/norm(e_t)^2) * cross(e_t, e_r)/norm(e_t);
e_SW_test_n = e_t / norm(e_t)^2 + sqrt(1 - 1/norm(e_t)^2) * cross(e_t, e_r)/norm(e_t);
k_x(e_SW_test_p, e_t, e_r)
k_x(e_SW_test_n, e_t, e_r)


function k_x = k_x(e_SW, e_t, e_r)
    k_r = cross(e_SW - e_t, e_r);
    k_x = cross(k_r, e_SW);
end