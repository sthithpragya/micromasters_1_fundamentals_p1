function [pos, R] = puma_fk(theta1, theta2, theta3, theta4, theta5, theta6)
%PUMA_FK The input to the function will be the joint angles of the robot in radians.
%    The output must contain end effector position of the robot arm and the rotation matrix representing the rotation from frame
%    6 to frame 0, as specified in the question.

    %% Your code goes here

    t01 = compute_dh_matrix(0, pi/2, 13, theta1);
    t12 = compute_dh_matrix(8, 0, -2.5, theta2);
    t23 = compute_dh_matrix(0, -pi/2, -2.5, theta3);
    t34 = compute_dh_matrix(0, pi/2, 8, theta4);
    t45 = compute_dh_matrix(0, -pi/2, 0, theta5);
    t56 = compute_dh_matrix(0, 0, 2.5, theta6);

    t06 = t01*t12*t23*t34*t45*t56;
    p06 = t06(1:3,4);
    pos = p06.';
    R = t06(1:3,1:3);
    
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
