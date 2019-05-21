function [ ik_sol ] = puma_ik_5( x, y, z, R )
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
    pwc = pe - R*2.5*[0;0;1]
    if pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 < 25 || pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 > 281
        ik_sol = [];
    else
        theta1 = asin(-5/sqrt(pwc(1)^2+pwc(2)^2))+atan2(pwc(2),pwc(1));
        X = (pwc(3)-13)/8;
        if cos(theta1(1)) == 0
            Y = pwc(2)/(8*sin(theta1));
        else
            Y = (pwc(1) + 5*sin(theta1))/(8*cos(theta1));
        end
        S = (2 - (X^2 + Y^2))/2;
        theta3 = asin(S);
        if S == 1
            theta2 = 0;
        else
            Z = (X^2 - Y^2)/(2*(1 - S)); 
            theta2 = (asin(Z)-theta3)/2;
        end
        %checking for the theta values
        
        THETA1 = theta1;
        THETA2 = theta2;
        THETA3 = theta3;
        
        t01 = compute_dh_matrix(0, pi/2, 13, THETA1);
        t12 = compute_dh_matrix(8, 0, -2.5, THETA2);
        t23 = compute_dh_matrix(8, -pi/2, -2.5, THETA3);
        t03 = t01*t12*t23;
        
        R03_ik = t03(1:3,1:3);        
        R36_ik = R03_ik.'*R;
        
        if R36_ik(3,3) == 1
            THETA5 = 0;
            THETA4 = 0;
            THETA6 = atan2(R36_ik(2,1), R36_ik(1,1));
        else
            THETA6 = atan2(-R36_ik(3,2),R36_ik(3,1));
            THETA4 = atan2(-R36_ik(2,3),-R36_ik(1,3));
            if R(1,3) == 0
                THETA5 = atan2(R(2,3)/(-sin(THETA4)),R(3,3));
            else
                THETA5 = atan2(R(1,3)/(-cos(THETA4)),R(3,3));
            end
        end
        ik_sol = [THETA1 THETA2 THETA3 THETA4 THETA5 THETA6];
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

function A = rowcheck(m1, r1, m2, r2)
    M = zeros(1,3);
    for i = 1:size(m1,2)
        if m1(r1,i) == m2(r2,i)
            M(i) = 1;
        else
            M(i) = 0;
        end
    end
    if M == [1 1 1]
        A = 1;
    else
        A = 0;
    end
end
