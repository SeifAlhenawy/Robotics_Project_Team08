% function q_deg = inverse_kinematics_func(q0, X_desired)
% q = q0;
% for k = 1:100
%     F = forward_kinematics_func(q);
%     e = X_desired - F;
% 
%     J_inv = inverse_jacobian_matrix2(q);
%     q = q + J_inv * e;
% end
% q_deg = rad2deg(q);
% q_deg = mod(q_deg, 360);
% 
% end

function q_deg = inverse_kinematics_func(q0, X_desired)
    % Iterative inverse kinematics using Jacobian pseudo-inverse
    %
    % Inputs:
    %   q0         - initial guess [4x1] in radians
    %   X_desired  - desired position [3x1]
    %
    % Output:
    %   q_deg      - joint angles [4x1] in degrees

    % Ensure column vector
    q = q0(:);
    X_desired = X_desired(:);

    max_iter = 1000;
    tol = 1e-3;
    alpha = 0.5;  % step size

    for k = 1:max_iter
        X_current = forward_kinematics_func(q);
        error = X_desired - X_current;

        if norm(error) < tol
            break;
        end

        J_inv = inverse_jacobian_matrix(q);  % 4x3 pseudo-inverse
        q = q + alpha * J_inv * error;        % update joint angles
    end

    q_deg = rad2deg(q);  % return as column vector in degrees
end

