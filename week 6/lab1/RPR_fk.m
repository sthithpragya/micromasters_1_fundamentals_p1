function [ pos, R ] = RPR_fk( theta1, d2, theta3 )
%RPR_FK Write your code here. The input to the function will be the joint
%    angles of the robot in radians, and the extension of the prismatic joint in inches.
%    The output includes: 
%    1) The position of the end effector and the position of 
%    each of the joints of the robot, as explained in the question.
%    2) The rotation matrix R_03, as explained in the question.

    %% YOUR CODE GOES HERE
    
    
    pos = zeros(4, 3);
    R = eye(3);
    R01 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1]*[1 0 0; 0 -1/sqrt(2) 1/sqrt(2); 0 -1/sqrt(2) -1/sqrt(2)];
    p01 = [0; 0; 10];
    
    R12 = [0 0 1; -1 0 0; 0 -1 0];
    p12 = [0; 0; d2];
    
    R23 = [cos(theta3) -sin(theta3) 0; sin(theta3) cos(theta3) 0; 0 0 1]*[0 -1/sqrt(2) 1/sqrt(2); 0 -1/sqrt(2) -1/sqrt(2); 1 0 0];
    p23 = [cos(theta3) -sin(theta3) 0; sin(theta3) cos(theta3) 0; 0 0 1]*[5/sqrt(2);-5/sqrt(2);0];
    
    R = R01*R12*R23;
    
    p02 = p01 + R01*p12;
    p03 = p02 + R01*R12*p23;
    
    pos(1, 1:3) = [0 0 0];
    pos(2, 1:3) = p01.';
    pos(3, 1:3) = p02.';
    pos(4, 1:3) = p03.';
    
end