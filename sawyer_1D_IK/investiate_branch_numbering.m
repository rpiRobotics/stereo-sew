kin = define_sawyer();

%%
%q = rand_angle([7 1]); 
q(2) = rand_angle;

q_range = linspace(0, 2*pi,500);

c_range = NaN(5, length(q_range));
for i = 1:length(q_range)
    for i_joint = 4
    %for i_joint = 2:6
        q_t = q;
        q_t(i_joint) = q_range(i);
        c_range(i_joint-1, i) = singularity_closeness(kin, q_t);
    end
end

hold on
plot(rad2deg(q_range), c_range);
yline(0);
legend(["2", "3", "4", "5", "6"])



function c = singularity_closeness(kin, q)
J = robotjacobian(kin, q);
singular_values = svd(J);
c = singular_values(end);
end