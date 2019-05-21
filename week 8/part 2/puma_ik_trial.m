[pos, R] = puma_fk(pi,pi/2,pi/2,-pi/2,-pi/6,pi/3)

ik_sol1 = puma_ik_6(pos(1), pos(2), pos(3), R)

[pos_ik, R_ik] = puma_fk(ik_sol1(1),ik_sol1(2),ik_sol1(3),ik_sol1(4),ik_sol1(5),ik_sol1(6))