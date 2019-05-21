qo = 0;
q = [1/sqrt(3);1/sqrt(3);1/sqrt(3)];
J = [0 -q(3) q(2);
    q(3) 0 -q(1);
    -q(2) q(1) 0];
% generating Rotation matrix 'R' from it
R = (qo^2 - q.'*q)*eye(3) + 2*qo*J + 2*q*q.';

theta = acos((trace(R)-1)/2)
B = [0;0;0];
x = linsolve((R-eye(3)),B)