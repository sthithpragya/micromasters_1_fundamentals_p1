syms t1 t2 t3

a = 8*cos(t1)*(cos(t2) + cos(t2+t3)) - 5*sin(t1);
b = 8*sin(t1)*(cos(t2) + cos(t2+t3)) + 5*cos(t1);
c = 8*(sin(t2) + sin(t2+t3)) + 13;

eq = [a b c];
val = [16 5 13];

sol1 = solve(eq == val, [t1 t2 t3]);
sol2 = struct2cell(sol1);


%T1 = vpa(sol2(1,1),3);
x = sol2(1,1);
y = vpa(x);
y(1)
y(2)
double(y(2))

xx = sol2(2,1);
yy = vpa(xx);
yy(1)
yy(2)
double(yy(2))

