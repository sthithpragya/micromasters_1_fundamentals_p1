function [ v06, w06 ] = puma_velocities( thetas, thetadot )
%PUMA_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x6 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x6 matrix
%    The output has 2 parts:
%    v06 - The linear velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    w06 - The angular velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk


    t01 = compute_dh_matrix(0, pi/2, 13, thetas(1));
    t12 = compute_dh_matrix(8, 0, -2.5, thetas(2));
    t23 = compute_dh_matrix(0, -pi/2, -2.5, thetas(3));
    t34 = compute_dh_matrix(0, pi/2, 8, thetas(4));
    t45 = compute_dh_matrix(0, -pi/2, 0, thetas(5));
    t56 = compute_dh_matrix(0, 0, 2.5, thetas(6));

    t02 = t01*t12;
    t03 = t02*t23;
    t04 = t03*t34;
    t05 = t04*t45;
    t06 = t05*t56;
    
    p01 = t01(1:3,4);
    p02 = t02(1:3,4);
    p03 = t03(1:3,4);
    p04 = t04(1:3,4);
    p05 = t05(1:3,4);
    p06 = t06(1:3,4);
    
    p16 = p06 - p01;
    p26 = p06 - p02;
    p36 = p06 - p03;
    p46 = p06 - p04;
    p56 = p06 - p05;
    
    z0 = [0; 0; 1];
    z1 = t01(1:3,3);
    z2 = t02(1:3,3);
    z3 = t03(1:3,3);
    z4 = t04(1:3,3);
    z5 = t05(1:3,3);
    z6 = t06(1:3,3);
    
    J1 = [cross(z0,p06); z0];
    J2 = [cross(z1,p16); z1];
    J3 = [cross(z2,p26); z2];
    J4 = [cross(z3,p36); z3];
    J5 = [cross(z4,p46); z4];
    J6 = [cross(z5,p56); z5];
    
    J = [J1 J2 J3 J4 J5 J6];

    V = J*thetadot.';
    
    v06 = V(1:3).';
    w06 = V(4:6).';
    
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
