syms x y t1
eq1 = x*sin(t1) - y*cos(t1) + 5

% sol = solve(eq1 == 0, t1)
t = asin(-5/sqrt(x^2+y^2)) + atan2(y,x);

sq2 = subs(eq1,t1,t)

