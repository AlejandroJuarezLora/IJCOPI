syms x1 x2 x3 x4;
%syms  mb mw l r Iw Ib g u;
syms   u;
mb = 1;
mw = 2;
r = 0.25;
l = 1.2;
Iw = 10;
Ib = 10;
g = 9.81;

%dinamica del sistema en espacio de estados
M = [(mb + mw) * r^2 + Iw mb* l * r * cos(x3);
    mb* l * r * cos(x3) mb* l^2 + Ib];

F = [u + mb * l * r * x4^2 * sin(x3);
     -u + mb * g * l * sin(x3)];
 
 ddq = inv(M) * F;
 
 
 %poniendo el sistema  en espacio de estados
 dx1 = x2;
 dx2 = simplify(ddq(1,1))
 dx3 = x4;
 dx4 = simplify(ddq(2,1))
 

 %obteniendo la matriz A de dx = Ax + Bu
Asym = jacobian([dx1 dx2 dx3 dx4], [x1 x2 x3 x4]);
A = subs(Asym, [x1 x2 x3 x4], [0 0 0 0])

%obteniendo la matriz B de dx = Ax + Bu
Bsym = jacobian([dx1 dx2 dx3 dx4], [u]);
B = subs(Bsym, [x1 x2 x3 x4 ], [0 0 0 0])
    
%eigenvalores del sistema
lambda = double(eig(A)) %verificando que el sistema es inestable
rank(ctrb(A,B))         %si la matriz de controlabilidad es de rango completo, el sist es controlable
% 
% %  Design LQR controller
Q = eye(4); % 4x4 identify matrix 
R = .0001;
K = lqr(double(A),double(B),Q,R)
% 
% 
tspan = 0:.001:20;
x0 = [4.;0; 0.2; 0]; % initial condition 
wr = [6.; 0; 0; 0.]; % reference position
u=@(x)-K*(x - wr); % control law
% 
[t,x] = ode45(@(t,x)segway(x, mb, mw, l, r, Iw, Ib, g, u(x)),tspan,x0);
% 
figure
plot(t,x,'linewidth',2);
legend('\theta_w','\omega_w','\theta_b','\omega_b')
% 
% 
uq1 = []
udq1 = []
uq2 = []
udq2 = []

for i = 1:length(x)
     xprov = [x(i,1); x(i,2); x(i,3); x(i,4)];
     uq1 = [uq1 -K(1) * (x(i,1) - wr(1))];
     udq1 = [udq1 -K(2) * (x(i,2) - wr(2))];
     uq2 = [uq2 -K(3) * (x(i,3) - wr(3))];
     udq2 = [udq2 -K(4) * (x(i,4) - wr(4))];
end
 
figure
plot(t,uq1,t,udq1,t,uq2,t,udq2,'linewidth',2);
legend('u_{q_1}','u_{dq_1}','u_{q_2}','u_{q_2}')

figure
plot(t,uq1 + udq1 + uq2 + udq2,'linewidth',2);
legend('u')


function dx = segway(x, mb, mw, l, r, Iw, Ib, g, u)
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);
    
    dx(1,1) = x2;
    dx(2,1) = (r*sin(x3)*l^3*mb^2*x4^2 - g*r*cos(x3)*sin(x3)*l^2*mb^2 + u*l^2*mb + Ib*r*sin(x3)*l*mb*x4^2 + r*u*cos(x3)*l*mb + Ib*u)/(- l^2*mb^2*r^2*cos(x3)^2 + l^2*mb^2*r^2 + mw*l^2*mb*r^2 + Iw*l^2*mb + Ib*mb*r^2 + Ib*mw*r^2 + Ib*Iw);
    dx(3,1) = x4;
    dx(4,1) = -(cos(x3)*sin(x3)*l^2*mb^2*r^2*x4^2 - g*sin(x3)*l*mb^2*r^2 - g*mw*sin(x3)*l*mb*r^2 + u*cos(x3)*l*mb*r - Iw*g*sin(x3)*l*mb + u*mb*r^2 + mw*u*r^2 + Iw*u)/(- l^2*mb^2*r^2*cos(x3)^2 + l^2*mb^2*r^2 + mw*l^2*mb*r^2 + Iw*l^2*mb + Ib*mb*r^2 + Ib*mw*r^2 + Ib*Iw);
    
end


