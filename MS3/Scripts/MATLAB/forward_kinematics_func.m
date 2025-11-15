function X = forward_kinematics_func(q)
L1=43.55;
L2=139.55;
L3=139.75;
L4 =10;
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

T01=[cos(q1) -sin(q1)*cos(pi/2) sin(q1)*sin(pi/2) 0;
     sin(q1)  cos(q1)*cos(pi/2) -cos(q1)*sin(pi/2) 0;
     0        sin(pi/2)          cos(pi/2)         L1;
     0 0 0 1];

T12=[cos(pi/2+q2) -sin(pi/2+q2)*cos(pi) sin(pi/2+q2)*sin(pi) L2*cos(pi/2+q2);
     sin(pi/2+q2)  cos(pi/2+q2)*cos(pi) -cos(pi/2+q2)*sin(pi) L2*sin(pi/2+q2);
     0             sin(pi)              cos(pi)               0;
     0 0 0 1];

T23=[cos(q3) -sin(q3)*cos(pi) sin(q3)*sin(pi) L3*cos(q3);
     sin(q3)  cos(q3)*cos(pi) -cos(q3)*sin(pi) L3*sin(q3);
     0        sin(pi)          cos(pi)         0;
     0 0 0 1];

T34=[cos(q4) -sin(q4)*cos(0) sin(q4)*sin(0) L4*cos(q4);
     sin(q4)  cos(q4)*cos(0) -cos(q4)*sin(0) L4*sin(q4);
     0        sin(0)          cos(0)         0;
     0 0 0 1];
T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 * T34;

T04=T01*T12*T23*T34;
X=T04(1:3,4);
end