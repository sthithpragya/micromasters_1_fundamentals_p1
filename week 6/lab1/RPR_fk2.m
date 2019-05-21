function [ pos, R ] = RPR_fk2( theta1, d2, theta3 )
%RPR_FK Write your code here. The input to the function will be the joint
%    angles of the robot in radians, and the extension of the prismatic joint in inches.
%    The output includes: 
%    1) The position of the end effector and the position of 
%    each of the joints of the robot, as explained in the question.
%    2) The rotation matrix R_03, as explained in the question.

    %% YOUR CODE GOES HERE
    
    
    pos = zeros(4, 3);
    R = eye(3);
    R1 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1]*[1 0 0; 0 -1/sqrt(2) 1/sqrt(2); 0 -1/sqrt(2) -1/sqrt(2)];
    p1 = [0; 0; 10];
    
    R2 = [1 0 1; 0 0 0; 0 -1 0];
    p2 = [0; 0; d2];
    
    R3 = [cos(theta3) -sin(theta3) 0; sin(theta3) cos(theta3) 0; 0 0 1]*[1 -1/sqrt(2) 1/sqrt(2); 0 -1/sqrt(2) -1/sqrt(2); 0 0 0];
    p3 = [5/sqrt(2) -5/sqrt(2) 0];
    
    T01 = zeros(4,4);
    T01(1:3, 1:3) = R1;
    T01(1:3, 4) = p1;
    T01(4, 1:4) = [0 0 0 1];
    
    T12 = zeros(4,4);
    T12(1:3, 1:3) = R2;
    T12(1:3, 4) = p2;
    T12(4, 1:4) = [0 0 0 1];
    
    T23 = zeros(4,4);
    T23(1:3, 1:3) = R3;
    T23(1:3, 4) = p3;
    T23(4, 1:4) = [0 0 0 1];
    
    T02 = T01*T12;
    T03 = T02*T23;
    
    R = T03(1:3,1:3);
    pos(1,1:3) = [0 0 0];
    pos(2,1:3) = T01(1:3,4).';
    pos(3,1:3) = T02(1:3,4).';
    pos(4,1:3) = T03(1:3,4).';
    
    
    
    

end