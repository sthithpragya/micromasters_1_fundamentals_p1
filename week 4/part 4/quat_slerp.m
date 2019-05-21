function [ q_int ] = quat_slerp( q0, q1, steps )
%QUAT_SLERP Perform SLERP between two quaternions and return the intermediate quaternions
%   Usage: [ q_int ] = quat_slerp( q0, q1, steps )
%   Inputs:
%       q0 is the quaternion representing the starting orientation, 1x4 matrix
%       q1 is the quaternion representing the final orientation, 1x4 matrix
%       steps is the number of intermediate quaternions required to be returned, integer value
%       The first step is q0, and the last step is q1
%   Output:
%       q_int contains q0, steps-2 intermediate quaternions, q1
%       q_int is a (steps x 4) matrix

    %% Your code goes here
    q_int = zeros(steps, 4);
    dotprod = q0(1)*q1(1)+q0(2)*q1(2)+q0(3)*q1(3)+q0(4)*q1(4);
    if dot < 0
        q1 = -q1;
        dotprod = q0(1)*q1(1)+q0(2)*q1(2)+q0(3)*q1(3)+q0(4)*q1(4);
    else
        ;
    end
    omega = acos(dotprod);
    for i = 1:steps
        t = (i-1)/(steps-1);
        q_int(i,:) = (sin(omega*(1-t))/sin(omega))*q0 + (sin(omega*t)/sin(omega))*q1;
    end
end