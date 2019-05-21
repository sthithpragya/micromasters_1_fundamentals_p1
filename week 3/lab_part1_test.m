q1=rand(1,4);
q1=q1/norm(q1);
q2=rand(1,4);
q2=q2/norm(q2);
q = lab_part1(q1,q2)
a = quatmultiply(q1,q2)