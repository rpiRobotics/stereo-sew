function [q, is_LS] = q_given_q12345(q12345, kin, R)
if isfield(kin, 'RT')
    R = R * kin.RT';
end

R_01 = rot(kin.H(:,1), q12345(1));
R_12 = rot(kin.H(:,2), q12345(2));
R_23 = rot(kin.H(:,3), q12345(3));
R_34 = rot(kin.H(:,4), q12345(4));
R_45 = rot(kin.H(:,5), q12345(5));
R_05 = R_01*R_12*R_23*R_34*R_45;

p = kin.H(:,6); % Needs to be non-collinear to h_7

[q_7_solns, q_6_solns, is_LS] = subproblem.sp_2(p, R_05'*R*p, kin.H(:,7), -kin.H(:,6));

% Remove extraneous solution
R_56 = rot(kin.H(:,6), q_6_solns(1));
R_67 = rot(kin.H(:,7), q_7_solns(1));

R_07_test = R_05 * R_56 * R_67;

if norm(R_07_test - R) < 1e-6
    q = [q12345; q_6_solns(1); q_7_solns(1)]
else
    q = [q12345; q_6_solns(2); q_7_solns(2)]
end

end