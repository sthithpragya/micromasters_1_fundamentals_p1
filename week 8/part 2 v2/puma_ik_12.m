function [ ik_sol ] = puma_ik_12( x, y, z, R )
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
    pe = [x;y;z];

    %pwc - position of wrist center
    pwc = pe - R*10.5*[0;0;1];
    if pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 < 25 || pwc(1)^2 + pwc(2)^2 + (pwc(3)-13)^2 > 281
        ik_sol = [];
    else
        t1 = [asin(-5/sqrt(pwc(1)^2+pwc(2)^2))+atan2(pwc(2),pwc(1));
            pi - asin(-5/sqrt(pwc(1)^2+pwc(2)^2))+atan2(pwc(2),pwc(1))];
        X = (pwc(3)-13)/8;
        Y = [(pwc(1) + 5*sin(t1(1)))/(8*cos(t1(1)));
            (pwc(1) + 5*sin(t1(2)))/(8*cos(t1(2)))];
            
        t2minus3 = [asin((X^2+Y(1)^2-2)/2);asin((X^2+Y(2)^2-2)/2)];
            
        K = [sin(-t2minus3(1)); sin(-t2minus3(2))];
            
        Z = [(Y(1)^2 - X^2)/(2*(K(1)-1));(Y(2)^2 - X^2)/(2*(K(2)-1))];
        t3plus2 = [asin(Z(1));asin(Z(2))];

        a = t2minus3;
        b = t3plus2;
        
        t2 = [(a(1)+b(1))/2, (pi+a(1)-b(1))/2, (pi+b(1)-a(1))/2, pi-(a(1)+b(1))/2;
            (a(2)+b(2))/2, (pi+a(2)-b(2))/2, (pi+b(2)-a(2))/2, pi-(a(2)+b(2))/2];
            
        t3 = [(b(1)-a(1))/2, (a(1)+b(1)-pi)/2, (pi-a(1)-b(1))/2, (a(1)-b(1))/2;
            (b(2)-a(2))/2, (a(2)+b(2)-pi)/2, (pi-a(2)-b(2))/2, (a(2)-b(2))/2];
        
        pwc1 = [];
        
        for i1 = 1:2
            for i2 = 1:4
                for i3 = 1:4          
                    T01 = compute_dh_matrix(0, pi/2, 13, t1(i1));
                    T12 = compute_dh_matrix(8, 0, -2.5, t2(i1,i2));
                    T23 = compute_dh_matrix(0, -pi/2, -2.5, t3(i1,i3));
                    T03 = T01*T12*T23;
                    pwc
                    pwc2 = T03(1:3,4)                  
                    if round(pwc2,5) == round(pwc,5)
                            pwc1 = [pwc1; [i1 i2 i3]];
                    end
                end
            end
        end    
        
        pwc1

        theta1 = t1(pwc1(1,1));
        theta2 = t2(pwc1(1,2));
        theta3 = t3(pwc1(1,3));

        t01 = compute_dh_matrix(0, pi/2, 13, theta1);
        t12 = compute_dh_matrix(8, 0, -2.5, theta2);
        t23 = compute_dh_matrix(0, -pi/2, -2.5, theta3);
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