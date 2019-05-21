syms t1 t2 t3

eq1 = -5*sin(t1) + 8*cos(t1)*(cos(t2) - sin(t2+t3)) - 8;
eq2 = 5*cos(t1) + 8*sin(t1)*(cos(t2) - sin(t2+t3)) + 5;
eq3 = 13 + 8*(sin(t2) + cos(t2+t3)) - 21;

eq = [eq1; eq2; eq3];
val = [0; 0; 0];
sol1 = solve(eq == val, [t1 t2 t3]);
sol2 = struct2cell(sol1);

T1 = vpa(sol2(1));
T1(1)
vpa(cos(T1(1)))

