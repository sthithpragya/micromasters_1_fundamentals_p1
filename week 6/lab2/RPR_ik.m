function [ ik_sol ] = RPR_ik( x, y, z, R )
%RPR_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_30 as described in the question.
%    The output must be the joint angles and extensions of the robot to achieve 
%    the end effector position and orientation.

    %% YOUR CODE GOES HERE
    
    if z > 10+5*sqrt(2)
        if x^2 + y^2 - (z-(10+5*sqrt(2)))^2 < 0
            ik_sol = [];
        else
            ik_sol = invkin(x,y,z,R);
        end
    elseif z < 10-5*sqrt(2)
        if x^2 + y^2 - (z-(10-5*sqrt(2)))^2 < 0
            ik_sol = [];
        else
            ik_sol = invkin(x,y,z,R);
        end
    else
        ik_sol = invkin(x,y,z,R);
    end
     
end

function [ joint_var ] = invkin(x,y,z,R)  
    theta1 = atan2(R(2,1),R(1,1));
    theta3 = atan2(R(3,3),-R(3,2));
    d2 = -sqrt(2)*(z-10-5*sin(theta3));
    joint_var = [theta1 d2 theta3];
end