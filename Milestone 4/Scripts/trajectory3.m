%% 1. Define time and trajectory
 t = 0:0.1:10;                  % 11 points, 1s step
% x =  50+ 10*t;               % x goes from 100 to 150
% y = 100*ones(size(t));       % constant y
% z = 100*ones(size(t));   
 x = -100+100*cos(0.2*pi*t) ;              % x goes from 100 to 150
 z =-100+100*sin(0.2*pi*t) ;      % constant y
 y = 50*ones(size(t));  
 
pos = [x; y; z];             % 3x11, each column = [x;y;z] position

% Preallocate joint matrix
Q = zeros(4, size(pos,2));

% Initial guess for IK
q0 = [0; 0; 0; 0];

% Solve IK for each point in the trajectory
for i = 1:size(pos,2)
    Q(:,i) = inverse_kinematics_func(q0, pos(:,i));
    q0 = deg2rad(Q(:,i));   % use last solution as next initial guess
end

result = zeros(3, size(Q,2));

for i = 1:size(Q,2)
    result(:,i) = forward_kinematics_func(deg2rad(Q(:,i)));
end
x1=result(1,:);
y1=result(2,:);
z1=result(3,:);

% Extract joint angles
q1 = Q(1,:);
q2 = Q(2,:);
q3 = Q(3,:);
q4 = Q(4,:);
q1 = mod(q1, 360);
q2 = mod(q2, 360);
q3 = mod(q3, 360);
q4 = mod(q4, 360);

q = [q1; q2; q3; q4];      % 4x11
q = deg2rad(q);             % convert to radians

% Compute end-effector trajectory
X = zeros(3, size(q,2));   % 3x11
for i = 1:size(q,2)
    X(:,i) = forward_kinematics_func(q(:,i));  % each column is end-effector position
end

% Prepare signals for plotting or simulation
q1_signal = [t' q1'];
q2_signal = [t' q2'];
q3_signal = [t' q3'];
q4_signal = [t' q4'];



X = zeros(3, size(q,2));   % 3x11
for i = 1:size(q,2)
    X(:,i) = forward_kinematics_func(q(:,i));  % each column is end-effector position
end

x1=result(1,:);
y1=result(2,:);
z1=result(3,:);

% Prepare signals for plotting or simulation
q1_signal = [t' q1'];
q2_signal = [t' q2'];
q3_signal = [t' q3'];
q4_signal = [t' q4'];
figure;
plot3(round(x1,1), round(y1,1), round(z1,1), 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Line Plot');
view(3);
