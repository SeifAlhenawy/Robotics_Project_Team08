import sympy as sp
import numpy as np

# Define symbolic joint variables
q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4')

# Define link lengths (in mm)
L1 = 38.55   # Link 1
L2 = 140     # Link 2
L3 = 103     # Link 3
L4 = 29.85   # End-effector link

# Define rotation angles
pi = sp.pi

# Transformation Matrices

# Base to Joint 1 
T01 = sp.Matrix([
    [sp.cos(q1), -sp.sin(q1)*sp.cos(pi/2), sp.sin(q1)*sp.sin(pi/2), 0],
    [sp.sin(q1),  sp.cos(q1)*sp.cos(pi/2), -sp.cos(q1)*sp.sin(pi/2), 0],
    [0,           sp.sin(pi/2),             sp.cos(pi/2),            L1],
    [0,           0,                        0,                       1]
])

# Joint 1 to Joint 2 
T12 = sp.Matrix([
    [sp.cos(pi/2 +q2), -sp.sin(pi/2 + q2)*sp.cos(pi), sp.sin(pi/2+ q2)*sp.sin(pi), L2*sp.cos(pi/2+ q2)],
    [sp.sin(pi/2 +q2),  sp.cos(pi/2 + q2)*sp.cos(pi), -sp.cos(pi/2 + q2)*sp.sin(pi), L2*sp.sin(pi/2+ q2)],
    [0,           sp.sin(pi),             sp.cos(pi),            0],
    [0,           0,                      0,                     1]
])

# Joint 2 to Joint 3
T23 = sp.Matrix([
    [sp.cos(q3), -sp.sin(q3)*sp.cos(pi), sp.sin(q3)*sp.sin(pi), L3*sp.cos(q3)],
    [sp.sin(q3),  sp.cos(q3)*sp.cos(pi), -sp.cos(q3)*sp.sin(pi), L3*sp.sin(q3)],
    [0,           sp.sin(pi),             sp.cos(pi),            0],
    [0,           0,                      0,                     1]
])

# Joint 3 to Joint 4
T34 = sp.Matrix([
    [sp.cos(q4), -sp.sin(q4)*sp.cos(0), sp.sin(q4)*sp.sin(0), L4*sp.cos(q4)],
    [sp.sin(q4),  sp.cos(q4)*sp.cos(0), -sp.cos(q4)*sp.sin(0), L4*sp.sin(q4)],
    [0,           sp.sin(0),             sp.cos(0),            0],
    [0,           0,                     0,                   1]
])

# Full Transformation to End-Effector
T04 = sp.simplify(T01 * T12 * T23 * T34)

# Example joint angles (in radians)
angle_values = {q1: 0, q2: 90, q3: 0, q4: 0}
T04_s = T04.subs(angle_values)

# Convert to numeric (numpy array)
T04_num = np.array(T04_s.evalf(), dtype=float)

# Display results
print('Numeric Transformation Matrix (T04):')
print(np.round(T04_num, 4))  # rounded for better display

# End-effector XYZ position
P04 = T04_num[0:3, 3]
print('End-Effector Position [x y z] (mm):')
print(np.round(P04, 4))
