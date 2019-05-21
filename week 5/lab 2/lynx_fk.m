function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    
    pos = zeros(10, 3);
    
    t01 = compute_dh_matrix(0, -pi/2, 3, theta1);
    
    t12 = compute_dh_matrix(5.75, 0, 0, theta2-pi/2);
    t23 = compute_dh_matrix(7.375, 0, 0, theta3+pi/2);
    t34 = compute_dh_matrix(0, pi/2, 0, theta4+pi/2);
    t45 = compute_dh_matrix(0, 0, 4.125, pi+theta5);
    
    t02 = t01*t12;
    t03 = t02*t23;
    t04 = t03*t34;
    t05 = t04*t45;
    
    p00 = [0 0 0 1];
    p01 = t01(1:4,4).';
    p02 = t02(1:4,4).';
    p03 = t03(1:4,4).';
    p04 = t04(1:4,4).';
    p05 = t05(1:4,4).';
        
    ee1 = (t05*[0;0;-1.125;1]).';
    ee2 = (t05*[g/2;0;-1.125;1]).';
    ee3 = (t05*[-g/2;0;-1.125;1]).';
    ee4 = (t05*[g/2;0;0;1]).';
    ee5 = (t05*[-g/2;0;0;1]).';
    
    pos(1,:) = p00(1:3);
    pos(2,:) = p01(1:3);
    pos(3,:) = p02(1:3);
    pos(4,:) = p03(1:3);
    pos(5,:) = p04(1:3);
    
    pos(6,:) = ee1(1:3);
    pos(7,:) = ee2(1:3);
    pos(8,:) = ee3(1:3);
    pos(9,:) = ee4(1:3);
    pos(10,:) = ee5(1:3);
    
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