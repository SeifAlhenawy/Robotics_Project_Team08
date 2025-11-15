 function V_F = forward_velocity_kinematics(q, q_dot)

J = jacobian_matrix2(q);
V_F = J * q_dot;  % [vx; vy; vz]
end
