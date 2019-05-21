function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    
    pos = zeros(10, 4);
    
    t01 = compute_dh_matrix(0, -pi/2, 3, theta1);
    t12 = compute_dh_matrix(5.75, 0, 0, theta2-pi/2);
    t23 = compute_dh_matrix(7.375, 0, 0, theta3+pi/2);
    t34 = compute_dh_matrix(0, -pi/2, 0, theta4-pi/2);
    t45 = compute_dh_matrix(0, 0, 4.125, theta5);
    
    t02 = t01*t12;
    t03 = t02*t23; 
    t04 = t03*t34;
    t05 = t04*t45;
    
    p000 = [0 0 0 1];
    p001 = [0 0 3 1];
    p112 = [0 -5.75 0 1];
    p223 = [0 7.375 0 1];
    p334 = [0 0 0 1];
    p445 = [0 0 4.125 1];
    
    p012 = t01*p112.';
    p023 = t02*p223.';
    p034 = t03*p334.';
    p045 = t04*p445.';
    
    p002 = p001 + p012;
    p003 = p002 + p023;
    p004 = p003 + p034;
    
    p005 = p004 + p045;
    
    
    ee1 = (p005.' + t05*[0;0;-1.125;1]).';
    ee2 = (p005.' + t05*[g/2;0;-1.125;1]).';
    ee3 = (p005.' + t05*[-g/2;0;-1.125;1]).';
    ee4 = (p005.' + t05*[g/2;0;0;1]).';
    ee5 = (p005.' + t05*[-g/2;0;0;1]).';
    
    pos(1,:) = p000;
    pos(2,:) = p001;
    pos(3,:) = p002;
    pos(4,:) = p003;
    pos(5,:) = p004;
    
    pos(6,:) = ee1;
    pos(7,:) = ee2;
    pos(8,:) = ee3;
    pos(9,:) = ee4;
    pos(10,:) = ee5;
    
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