syms theta1 theta2 theta3 theta4 theta5 theta6 s1 s2 s3 s4 s5 s6 c1 c2 c3 c4 c5 c6 x y z 
hee = sym('hee%d%d', [4 4]);

t01 = compute_dh_matrix(0, pi/2, 13, theta1);
t12 = compute_dh_matrix(8, 0, -2.5, theta2);
t23 = compute_dh_matrix(8, -pi/2, -2.5, theta3);
t34 = compute_dh_matrix(0, pi/2, 0, theta4);
t45 = compute_dh_matrix(0, -pi/2, 0, theta5);
t56 = compute_dh_matrix(0, 0, 2.5, theta6);
t03 = t01*t12*t23;
t36 = t34*t45*t56;
t06 = t03*t36;

a = [sin(theta1) sin(theta2) sin(theta3) sin(theta4) sin(theta5) sin(theta6);
    cos(theta1) cos(theta2) cos(theta3) cos(theta4) cos(theta5) cos(theta6)];
b = [s1 s2 s3 s4 s5 s6;
    c1 c2 c3 c4 c5 c6];

%hee - given transformation matrix of end effector wrt frame 0
hee(4,4) = 1;
hee(1,4) = x;
hee(2,4) = y;
hee(3,4) = z;
hee(4,1:3) = zeros(1,3);

%pe - postion of end effector 
pe = hee(1:3,4);

%R - rotation matrix from origin to end effector
R = hee(1:3,1:3);

%pwc - position of wrist center
pwc = pe - R*2.5*[0;0;1];

subs(t36(1:3,1:3), a, b)

