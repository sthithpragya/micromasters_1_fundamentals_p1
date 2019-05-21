function [ ik_sol ] = puma_ik_6( x, y, z, R )
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
        theta1 = [asin(-5/sqrt(pwc(1)^2+pwc(2)^2))+atan2(pwc(2),pwc(1)) pi-asin(-5/sqrt(pwc(1)^2+pwc(2)^2))+atan2(pwc(2),pwc(1))];        
        X = (pwc(3)-13)/8;
        if cos(theta1(1)) == 0
            Y1 = pwc(2)/(8*sin(theta1(1)));
        else
            Y1 = (pwc(1) + 5*sin(theta1(1)))/(8*cos(theta1(1)));
        end
        
        if cos(theta1(2)) == 0
            Y2 = pwc(2)/(8*sin(theta1(2)));
        else
            Y2 = (pwc(1) + 5*sin(theta1(2)))/(8*cos(theta1(2)));
        end
        
        Y = [Y1 Y2];
        
        S = [(2 - (X^2 + Y(1)^2))/2 (2 - (X^2 + Y(2)^2))/2];
        theta3 = [asin(S(1)) pi-asin(S(1)) asin(S(2)) pi-asin(S(2))];
        if S(1) == 1
            theta2_1 = [0 0 0 0];
        else
            Z = (X^2 - Y(1)^2)/(2*(1 - S(1))); 
            theta2_1 = [(asin(Z)-theta3(1))/2 (pi-asin(Z)-theta3(1))/2 (asin(Z)-theta3(2))/2 (pi-asin(Z)-theta3(2))/2];
        end
        
        if S(2) == 1
            theta2_2 = [0 0 0 0];
        else
            Z = (X^2 - Y(2)^2)/(2*(1 - S(2))); 
            theta2_2 = [(asin(Z)-theta3(3))/2 (pi-asin(Z)-theta3(3))/2 (asin(Z)-theta3(4))/2 (pi-asin(Z)-theta3(4))/2];        
        end
        
        theta2 = [theta2_1 theta2_2];

        %checking for the theta values
        theta1
        theta2
        theta3
        
        pwc1 = [];
        pwc11 = []; %%
        for i1 = 1:2
            for i2 = 1:8
                for i3 = 1:4          
                    pwc11 = [pwc11; -5*sin(theta1(i1)) + 8*cos(theta1(i1))*(cos(theta2(i2)) - sin(theta2(i2) + theta3(i3)))]; %%
                    if -5*sin(theta1(i1)) + 8*cos(theta1(i1))*(cos(theta2(i2)) - sin(theta2(i2) + theta3(i3))) == pwc(1)
                            pwc1 = [pwc1; [i1 i2 i3]];
                    else
                            pwc1 = [pwc1; []];
                    end
                end
            end
        end
        
        pwc11(1)
        pwc(1)
        pwc11(1) == pwc(1)
        
        pwc2 = [];
        pwc22 = []; %%
        for i1 = 1:2
            for i2 = 1:4
                for i3 = 1:2 
                    pwc22 = [pwc22; 5*cos(theta1(i1)) + 8*sin(theta1(i1))*(cos(theta2(i2)) - sin(theta2(i2) + theta3(i3)))]; %%
                    if 5*cos(theta1(i1)) + 8*sin(theta1(i1))*(cos(theta2(i2)) - sin(theta2(i2) + theta3(i3))) == pwc(2)
                            pwc2 = [pwc2; [i1 i2 i3]];
                    else
                            pwc2 = [pwc2; []];
                    end
                end
            end
        end
        
        pwc3 = [];
        pwc33 = [];
        for i1 = 1:2
            for i2 = 1:4
                for i3 = 1:2          
                    pwc33 = [pwc33; 13 + 8*(sin(theta2(i2)) + cos(theta2(i2) + theta3(i3)))]; %%
                    if 13 + 8*(sin(theta2(i2)) + cos(theta2(i2) + theta3(i3))) == pwc(3)
                            pwc3 = [pwc3; [i1 i2 i3]];
                    else
                            pwc3 = [pwc3; []];
                    end
                end
            end
        end
        pwc11
        pwc1
        pwc22
        pwc2
        pwc33
        pwc3
        theta123 = [];
        for i = 1:size(pwc1,1)
            for j = 1:size(pwc2,1)
                for k = 1:size(pwc3,1)
                    if rowcheck(pwc1,i,pwc2,j) == 1 && rowcheck(pwc2,j,pwc3,k) == 1
                        theta123 = [theta123; pwc3(k,1:3)];
                    else
                        theta123 = [theta123; []];
                    end
                end
            end
        end
        theta123
        THETA1 = theta1(theta123(1,1));
        THETA2 = theta2(theta123(1,2));
        THETA3 = theta3(theta123(1,3));
        
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
