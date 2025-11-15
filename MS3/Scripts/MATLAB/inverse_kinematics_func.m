function q_deg = inverse_kinematics_func(q0, X_desired)
q = q0;
for k = 1:100
    F = forward_kinematics_func(q);
    e = X_desired - F;
  
    J_inv = inverse_jacobian_matrix(q);
    q = q + J_inv * e;
end
q_deg = rad2deg(q);
q_deg = mod(q_deg, 360);

end