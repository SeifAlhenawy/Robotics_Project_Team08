clc; clear; close all;

% Define symbolic joint variables
syms q1 q2 q3 q4;

% Define link lengths (in mm)
L1 = 38.55;   % Link 1
L2 = 140;     % Link 2
L3 = 103;     % Link 3
L4 = 29.85;   % End-effector link

% Transformation Matrices

% Base to Joint 1 
T01 = [[cos(q1) (-sin(q1)*cos(pi/2))   (sin(q1)*sin(pi/2))    0];
       [sin(q1)  (cos(q1)*cos(pi/2))   (-cos(q1)*sin(pi/2))    0];
       [0        sin(pi/2)            cos(pi/2)           L1];
       [0        0       0      1]];

% Joint 1 to Joint 2 
T12 = [[cos(pi/2 +q2)  (-sin(pi/2 +q2)*cos(pi))   (sin(pi/2 +q2)*sin(pi))    (L2*cos(pi/2 +q2))];
       [sin(pi/2 +q2)   (cos(pi/2 +q2)*cos(pi))   (-cos(pi/2 +q2)*sin(pi))   (L2*sin(pi/2 +q2))];
       [0          sin(pi)             cos(pi)     0];
       [0        0        0      1]];

% Joint 2 to Joint 3 .
T23 = [[cos(q3)   (-sin(q3)*cos(pi))   (sin(q3)*sin(pi))   L3*cos(q3)];
       [sin(q3)    (cos(q3)*cos(pi))   (-cos(q3)*sin(pi))    L3*sin(q3)];
       [0          sin(pi)       cos(pi)      0];
       [0        0         0     1]];

% Joint 3 to Joint 4
T34 = [[cos(q4) (-sin(q4)*cos(0))   (sin(q4)*sin(0))   (L4*cos(q4))];
       [sin(q4)  (cos(q4)*cos(0))   (-cos(q4)*sin(0))   (L4*sin(q4))];
       [0        sin(0)           cos(0)    0];
       [0        0       0      1]];


% Full Transformation to End-Effector
T04 = simplify(T01 * T12 * T23 * T34 );

% Example joint angles (radians)
T04_s = subs(T04, {q1, q2, q3, q4}, {pi/6, pi/6, pi/6, pi/6});

% Convert to numeric
T04_num = double(T04_s);

% Display results
disp('Numeric Transformation Matrix (T04):');
disp(T04_num);

% End-effector XYZ position
P04 = T04_num(1:3,4);
disp('End-Effector Position [x y z] (mm):');
disp(P04');