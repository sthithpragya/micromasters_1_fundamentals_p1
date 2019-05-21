% random vector 'u'
u  = rand(3,1)*2-1;
% generating a random quaternion 'q'
a = rand(4,1);
a = a/norm(a);
qo = a(1);
q = a(2:4,1);
J = [0 -q(3) q(2);
    q(3) 0 -q(1);
    -q(2) q(1) 0];
% generating Rotation matrix 'R' from it
R = (qo^2 - q.'*q)*eye(3) + 2*qo*J + 2*q*q.';
% generating rotated vector 'v'
v = R*u;

%plot
% origin
plot3(0,0,0,'k.')
axis vis3d
axis off
hold on
% x axis
plot3([0,1],[0,0],[0,0],'r');
text(1,0,0,'x')
hold on
% y axis
plot3([0,0],[0,1],[0,0],'g');
text(0,1,0,'y')
hold on
% z axis 
plot3([0,0],[0,0],[0,1],'b');
text(0,0,1,'z')
hold on
% u vector
plot3([u(1),0],[u(2),0],[u(3),0],':k')
text(u(1),u(2),u(3),strcat('(',num2str(u(1),3),',',num2str(u(2),3),',',num2str(u(3),3),')'))
% v vector
plot3([v(1),0],[v(2),0],[v(3),0],'--k')
text(v(1),v(2),v(3),strcat('(',num2str(v(1),3),',',num2str(v(2),3),',',num2str(v(3),3),')'))

