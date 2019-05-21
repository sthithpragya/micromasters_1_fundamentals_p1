function [ ik_sol ] = puma_ik_7( x, y, z, R )
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
        syms t1 t2 t3
        eq1 = -5*sin(t1) + 8*cos(t1)*(cos(t2) - sin(t2+t3)) - 8;
        eq2 = 5*cos(t1) + 8*sin(t1)*(cos(t2) - sin(t2+t3)) + 5;
        eq3 = 13 + 8*(sin(t2) + cos(t2+t3)) - 21;
        eq = [eq1; eq2; eq3];
        val = [0; 0; 0];
        sol1 = solve(eq == val, [t1 t2 t3]);
        sol2 = struct2cell(sol1);
        T1 = vpa(sol2(1));
        T2 = vpa(sol2(3));
        T3 = vpa(sol2(3));
        THETA1 = T1(1);
        THETA2 = T2(1);
        THETA3 = T3(1);

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
            THETA5 = acos(R36_ik(3,3));
            THETA6 = atan2(-R36_ik(3,2),R36_ik(3,1));
            THETA4 = atan2(-R36_ik(2,3),-R36_ik(1,3));
        end
        ik_sol = [vpa(THETA1) vpa(THETA2) vpa(THETA3) vpa(THETA4) vpa(THETA5) vpa(THETA6)];
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