function [ ik_sol ] = puma_ik_20( x, y, z, R )
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

    %pwc - position of wrist center at intersection of dimension 'd' and 'e'
    pwc = pe - R*10.5*[0;0;1];
    if pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 < 25 || pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 > 281
        ik_sol = [];
    else
         
        theta2 = [asin(((pwc(3)-13)/8)); pi-asin(((pwc(3)-13)/8))];        
        theta1 = [asin(((8/5)*pwc(2)*cos(theta2(1))-pwc(1))/((64/5)*cos(theta2(1))*cos(theta2(1))+5)), asin(((8/5)*pwc(2)*cos(theta2(2))-pwc(1))/((64/5)*cos(theta2(2))*cos(theta2(2))+5));
            pi-asin(((8/5)*pwc(2)*cos(theta2(1))-pwc(1))/((64/5)*cos(theta2(1))*cos(theta2(1))+5)), pi-asin(((8/5)*pwc(2)*cos(theta2(2))-pwc(1))/((64/5)*cos(theta2(2))*cos(theta2(2))+5))];
        
        %pwc2 - position of wrist center at distance 'f' from end effector
        pwc2 = pe - R*2.5*[0;0;1];
        
        x_r = sqrt( pwc2(1)^2 + pwc2(2)^2 - 5^2 );
        theta3 = [asin(round((8^2 + 8^2 - x_r^2 - (pwc2(3) - 13)^2)/(2*8*8))), pi-asin(round((8^2 + 8^2 - x_r^2 - (pwc2(3) - 13)^2)/(2*8*8)))];
        
        theta1
        theta2
        theta3
        
        theta_checker = [];
        for i1 = 1:2
            for i2 = 1:2
                for i3 = 1:2
     
                    T01 = compute_dh_matrix(0, pi/2, 13, theta1(i1));
                    T12 = compute_dh_matrix(8, 0, -2.5, theta2(i2,i1));
                    T23 = compute_dh_matrix(0, -pi/2, -2.5, theta3(i3));
                    %taking arbitrary theta4 just to generate the pwc2
                    T34 = compute_dh_matrix(0, pi/2, 8, pi/6);

                    T04 = T01*T12*T23*T34;
                    
                    if T04(1:3,4) == pwc2
                            theta_checker = [theta_checker; [i1 i2 i3]];
                    end
                end
            end
        end
        
        THETA1 = theta1(theta_checker(1,1));
        THETA2 = theta2(theta_checker(1,2), theta_checker(1,1));
        THETA3 = t3(theta_checker(1,3));
        
        t01 = compute_dh_matrix(0, pi/2, 13, THETA1);
        t12 = compute_dh_matrix(8, 0, -2.5, THETA2);
        t23 = compute_dh_matrix(0, -pi/2, -2.5, THETA3);
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
        ik_sol = [THETA1 THETA2 THETA3 theta4 theta5 theta6];
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