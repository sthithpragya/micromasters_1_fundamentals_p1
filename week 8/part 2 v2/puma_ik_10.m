function [ ik_sol ] = puma_ik_2( x, y, z, R )
%PUMA_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_60 as described in the question.
%    The output must be the joint angles of the robot to achieve 
%    the desired end effector position and orientation.

    %% YOUR CODE GOES HERE
    
    %hee - given transformation matrix of end effector wrt frame 0
    hee = zeros(4,4);
    hee(1:3,1:3) = R;
    hee(1:3,4) = [x;y;z];
    hee(4,4) = 1;
    
    %pe - postion of end effector 
    pe = hee(1:3,4);

    %R - rotation matrix from origin to end effector
    R = hee(1:3,1:3);

    %pwc - position of wrist center
    pwc = pe - R*10.5*[0;0;1];
    if pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 < 25 || pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 > 281
        ik_sol = [];
    else
        t1 = asin(-5/sqrt(pwc(1)^2+pwc(2)^2))+atan2(pwc(2),pwc(1));        
        g = (pwc(3)-13)/8;
        if cos(t1) == 0
            k = pwc(2)/(8*sin(t1));
        else
            k = (pwc(1)+5*sin(t1))/(8*cos(t1));
        end
        
        theta2min3 = asin((k^2+g^2-2)/2);
        theta2plus3 = (asin((k^2-g^2)/(2*(sin(-theta2min3) - 1))))/2;
        
        theta2min3 = a;
        theta2plus3 = b;
        
        theta1 = [t1 pi-t1];
        theta2 = [(a+b)/2, (pi+a-b)/2, (pi+b-a)/2, pi-(a+b)/2];
        theta3 = [(b-a)/2, (a+b-pi)/2, (pi-a-b)/2, (a-b)/2];
        
        
        t01 = compute_dh_matrix(0, pi/2, 13, theta1(1));
        t12 = compute_dh_matrix(8, 0, -2.5, theta2(1));
        t23 = compute_dh_matrix(0, -pi/2, -2.5, theta3(1));
        t03 = t01*t12*t23;
        
        R03_ik = t03(1:3,1:3);        
        R36_ik = R03_ik.'*R;
        
        if R36_ik(3,3) == 1
            theta5 = 0;
            theta4 = 0;
            theta6 = atan2(R36_ik(2,1), R36_ik(1,1));
        else
            theta5 = acos(R36_ik(3,3));
            theta6 = atan2(-R36_ik(3,2),R36_ik(3,1));
            theta4 = atan2(-R36_ik(2,3),-R36_ik(1,3));
        end
        ik_sol = [theta1 theta2 theta3 theta4 theta5 theta6];
    end    
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
          0 cosd(alpha*(180/pi)) -sind(alpha*(180/pi)) 0;
          0 sind(alpha*(180/pi)) cosd(alpha*(180/pi)) 0;
          0 0 0 1];
    
    A = Rz*Tz*Tx*Rx; 
end