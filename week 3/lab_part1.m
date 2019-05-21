function q = lab_part1(q1,q2)
q = zeros(1,4);
u0 = q1(1);
v0 = q2(1);
u = q1(1,2:4).';
v = q2(1,2:4).';
q(1) = u0*v0 - u.'*v;
Q = u0*v + v0*u + cross(u.',v.').';
q(2) = Q(1);
q(3) = Q(2);
q(4) = Q(3);
end

