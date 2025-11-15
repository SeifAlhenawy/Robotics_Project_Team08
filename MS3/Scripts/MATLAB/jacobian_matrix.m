function J =jacobian_matrix(q)
eps = 1e-6;

F0 = forward_kinematics_func(q);
J = zeros(3,4);
for i = 1:4
    dq = zeros(4,1);
    dq(i) = eps;
    F1 = forward_kinematics_func(q + dq);
    J(:,i) = (F1 - F0) / eps;
end
end
