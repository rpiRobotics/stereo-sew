% outputs
%   R_T, p_T - rotation and translation of end effector
%   p_E, p_W - position of shoulder, elbow, and wrist
%   J_T, J_E, J_W - jacobian of end effector, elbow, and wrist


% input:    
% q
% kin
%                   .H = [h1, ..., hn] (3xn)
%                   .P = [p01, ..., p_{n-1,n},p_{n,T}] (3x(n+1))
%                   .joint_type = 1xn (0 for revolute, 1 for prismatic)
% .RT tool rotation matrix

function [R_T, p_T, p_E, p_W, J_T, J_E, J_W] = fwd_kin_diff_with_SEW(kin, q)

q_ELBOW = 4;
q_WRIST = 7;

n=length(kin.joint_type);

joint_type=kin.joint_type;

H=kin.H;
P=kin.P;
T=eye(4,4);
J=zeros(6,n);

for i=1:n
    h=H(1:3,i);
    if joint_type(i)==0
        % R_{i-1,i}=rot(h_i,q_i) and p_{i-1,i}=constant vector
        R=expm(hat(h)*q(i));p=P(1:3,i);zeros(1,3);
    else
        % R_{i-1,i}=I and p_{i-1,i}=p_{i-1,i}^0 + h_i q_i
        R=eye(3,3);p=P(1:3,i)+q(i)*h;
    end
    % partial Jacobian in the base frame 
    J=phi(eye(3,3),T(1:3,1:3)*p)*J;
    if joint_type(i)==0
        J(:,i)=[T(1:3,1:3)*h;zeros(3,1)];
    else
        J(:,i)=[zeros(3,1);T(1:3,1:3)*h];
    end
    T=T*[R p;zeros(1,3) 1];

    if i == q_ELBOW
        p_E = T(1:3, 4);
        J_E = J(4:6,:);
    elseif i == q_WRIST
        p_W = T(1:3, 4);
        J_W = J(4:6,:);
    end

end  

J_T=phi(eye(3,3),T(1:3,1:3)*P(:,n+1))*J;
T=T*[eye(3,3) P(:,n+1);0 0 0 1];
R_T = T(1:3, 1:3) * kin.RT;
p_T = T(1:3, 4);

end

function phimat=phi(R,p)
    phimat=[R zeros(3,3);-R*hat(p) R]; 
end

function khat = hat(k)
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end
