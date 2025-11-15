% jacobian_matrix2.m
function J = jacobian_matrix2(q)
    % Link lengths (in meters or mm)
    L1 = 43.55;
    L2 = 139.55;
    L3 = 139.75;
    L4 = 10;
    
    % Step size for numerical differentiation
    h = 1e-5;
    
    % Number of joints
    n = length(q);
    
    % Initialize Jacobian matrix (3x4 for a 4-joint manipulator with 3D position output)
    J = zeros(3, n); 
    
    % Loop over each joint to calculate the partial derivatives
    for i = 1:n
        % Perturb the i-th joint angle by a small amount (+h and -h)
        q_plus = q;
        q_minus = q;
        
        q_plus(i) = q_plus(i) + h;  % Add small change
        q_minus(i) = q_minus(i) - h;  % Subtract small change
        
        % Calculate the end-effector position for both perturbed configurations
        X_plus = forward_kinematics_func(q_plus);
        X_minus = forward_kinematics_func(q_minus);
        
        % Approximate the derivative using the central difference formula
        J(:, i) = (X_plus - X_minus) / (2*h);
    end
end
