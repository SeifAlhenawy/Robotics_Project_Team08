
function J_inv = inverse_jacobian_matrix2(q)
    J = jacobian_matrix2(q);  % 6x4
    J_inv = pinv(J);          % 4x6 pseudo-inverse
end
