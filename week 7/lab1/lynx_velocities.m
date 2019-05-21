function [ v05, w05 ] = lynx_velocities( thetas, thetadot )
%LYNX_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x5 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x5 matrix
%    The output has 2 parts:
%    v05 - The linear velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    w05 - The angular velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE    

    t01 = compute_dh_matrix(0, -pi/2, 3, thetas(1));
    t12 = compute_dh_matrix(5.75, 0, 0, thetas(2)-pi/2);
    t23 = compute_dh_matrix(7.375, 0, 0, thetas(3)+pi/2);
    t34 = compute_dh_matrix(0, pi/2, 0, thetas(4)+pi/2);
    t45 = compute_dh_matrix(0, 0, 4.125, pi+thetas(5));

    t02 = t01*t12;
    t03 = t02*t23;
    t04 = t03*t34;
    t05 = t04*t45;

    p01 = t01(1:3,4);
    p02 = t02(1:3,4);
    p03 = t03(1:3,4);
    p04 = t04(1:3,4);
    p05 = t05(1:3,4);
    
    p15 = p05 - p01;
    p25 = p05 - p02;
    p35 = p05 - p03;
    p45 = p05 - p04;
    
    z0 = [0; 0; 1];
    z1 = t01(1:3,3);
    z2 = t02(1:3,3);
    z3 = t03(1:3,3);
    z4 = t04(1:3,3);
    z5 = t05(1:3,3);
    
    J1 = [cross(z0,p05); z0];
    J2 = [cross(z1,p15); z1];
    J3 = [cross(z2,p25); z2];
    J4 = [cross(z3,p35); z3];
    J5 = [cross(z4,p45); z4];
    
    J = [J1 J2 J3 J4 J5];

    V = J*thetadot.';
    
    v05 = V(1:3).';
    w05 = V(4:6).';

end

function A = compute_dh_matrix(r, alpha, d, theta)
    Rz = [cos(theta) -sin(theta) 0 0;
          sin(theta) cos(theta) 0 0;
          0 0 1 0;
          0 0 0 1];
    
    Tz = [1 0 0 0;
          0 1 0 0;
          0 0 1 d;
          0 0 0 1];
    
    Tx = [1 0 0 r;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];
    
    Rx = [1 0 0 0;
          0 cos(alpha) -sin(alpha) 0;
          0 sin(alpha) cos(alpha) 0;
          0 0 0 1];
    
    A = Rz*Tz*Tx*Rx; 
end