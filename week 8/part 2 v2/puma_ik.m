function [ ik_sol ] = puma_ik( x, y, z, R )
%PUMA_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_60 as described in the question.
%    The output must be the joint angles of the robot to achieve 
%    the desired end effector position and orientation.

    %% YOUR CODE GOES HERE
    syms theta1 theta2 theta3
    
    t01 = compute_dh_matrix(0, pi/2, 13, theta1);
    t12 = compute_dh_matrix(8, 0, -2.5, theta2);
    t23 = compute_dh_matrix(0, -pi/2, -2.5, theta3);
    t03 = t01*t12*t23;
    
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
        sol1 = solve(t03(1:3,4) == pwc, [theta1 theta2 theta3]);
        sol2 = struct2cell(sol1);
        t1 = vpa(sol2(1));
        t2 = vpa(sol2(2));
        t3 = vpa(sol2(3));
        
%         T=struct2cell(sol1);
%         m=cat(1,T{:});
%         disp(m);
        
        %calculated thetas 1 through 3 using wrist position
        theta1_ik = t1(1);
        theta2_ik = t2(1);
        theta3_ik = t3(1);

        R03_ik = subs(t03(1:3,1:3), [theta1 theta2 theta3], [theta1_ik theta2_ik theta3_ik]);
        R36_ik = R03_ik.'*R;

        if R36_ik(3,3) == 1
            theta5_ik = 0;
            theta4_ik = 0;
            theta6_ik = atan2(R36_ik(2,1), R36_ik(1,1));
        else
            theta5_ik = acos(R36_ik(3,3));
            theta6_ik = atan2(-R36_ik(3,2),R36_ik(3,1));
            theta4_ik = atan2(-R36_ik(2,3),-R36_ik(1,3));
        end
        
        a1 = double(theta1_ik);
        a2 = double(theta2_ik);
        a3 = double(theta3_ik);
        a4 = double(theta4_ik);
        a5 = double(theta5_ik);
        a6 = double(theta6_ik);
        ik_sol = [a1 a2 a3 a4 a5 a6];
        

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