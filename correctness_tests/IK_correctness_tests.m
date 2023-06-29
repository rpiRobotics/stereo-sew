% Check correctness of robot IK solutions when exact solution exists
setups = {
    SEW_IK_setups.IK_2R_2R_3R
    SEW_IK_setups.IK_3R_R_3R
    SEW_IK_setups.IK_R_R_3R_2R
    SEW_IK_setups.IK_2R_3R_2R
    SEW_IK_setups.IK_2R_3Rp_2R
    SEW_IK_setups.IK_R_R_3Rp_2R
};

%% Just 1 test
setup = SEW_IK_setups.IK_R_R_3R_2R;

[P, S_given] = setup.setup();
setup.error(P,S_given) % Make sure S_given is correct

S = setup.run(P);
%S = setup.run_mex(P);

S.is_LS
    
[e, e_R, e_T, e_psi] = setup.error(P,S);
e

%S_exact.Q = S.Q(:,~S.is_LS);
%e = setup.error(P, S_exact)
width(S.Q)
%% Multiple test
N_trials = 100;
min_errors = NaN(length(setups), N_trials);

for i = 1:length(setups)
    disp("Setup "+ i +" / " + length(setups))
    for j = 1:N_trials
        setup = setups{i};
        [P, S_given] = setup.setup();
        
        % S = setup.run_mex(P);
        S = setup.run(P);
        e = robot_IK_error(P,S);
        min_errors(i,j) = min([e NaN]);    
    end
end

%% 

mean(min_errors')
max(min_errors')

for i = 1:length(setups)
    histogram(min_errors(i,:)); hold on
end
hold off
% set(gca,'XScale','log')

%%
for i = 1:length(setups)
    semilogy(sort(min_errors(i,:))); hold on
end
hold off
legend(string(1:length(setups)))