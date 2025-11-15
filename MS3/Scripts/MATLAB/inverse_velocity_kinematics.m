function q_dot = inverse_velocity_kinematics(q, V_F) 
 J_inv = inverse_jacobian_matrix2(q);
 q_dot= J_inv * V_F;

end
% function q_dot = inverse_velocity_kinematics(q, V_F)
%     % Compute the Jacobian matrix
%     J = jacobian_matrix(q);
% 
%     % Compute the inverse of the Jacobian (assuming it's invertible)
%     J_inv = pinv(J);  % Pseudo-inverse in case Jacobian is singular
% 
%     % Compute the joint velocities
%     q_dot = J_inv * V_F;
% end
