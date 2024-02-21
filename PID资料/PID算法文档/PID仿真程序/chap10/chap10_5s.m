function [sys,x0,str,ts] = spacemodel(t,x,u,flag)

switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys=mdlOutputs(t,x,u)
R1=u(1);
dr1=-pi*sin(pi*t);
ddr1=-pi^2*cos(pi*t);
R2=u(2);
dr2=pi*cos(pi*t);
ddr2=-pi^2*sin(pi*t);

x(1)=u(3);
x(2)=u(4);
x(3)=u(5);
x(4)=u(6);

e1=x(1)-R1;
e2=x(3)-R2;
de1=x(2)-dr1;
de2=x(4)-dr2;

v=13.33;
q1=8.98;
q2=8.75;
g=9.8;

M=[v+q1+2*q2*cos(x(3)) q1+q2*cos(x(3));
   q1+q2*cos(x(3)) q1];
C=[-q2*x(4)*sin(x(3)) -q2*(x(2)+x(4))*sin(x(3));
    q2*x(2)*sin(x(3))  0];
G=[15*g*cos(x(1))+8.75*g*cos(x(1)+x(3));
   8.75*g*cos(x(1)+x(3))];
d1=2;d2=3;d3=6;

gama=1.5;
y1=de1+gama*e1;
y2=de2+gama*e2;

u1=-(d1+d2*norm([e1,e2])+d3*norm([de1,de2]))*sign(y1);
u2=-(d1+d2*norm([e1,e2])+d3*norm([de1,de2]))*sign(y2);

dr=[dr1;dr2]-gama*[e1;e2];
ddr=[ddr1;ddr2]-gama*[de1;de2];

Kp1=[180,0;0,190];
Kp2=[150,0;0,150];
Kv1=[180,0;0,180];
Kv2=[150,0;0,150];

alfa1=1;alfa2=1;
beta1=1;beta2=1;

fy=M*ddr+C*dr+G;
tol(1)=-(Kp1(1,1)+Kp2(1,1)/(alfa1+abs(e1)))*e1-(Kv1(1,1)+Kv2(1,1)/(beta1+abs(de1)))*de1+fy(1)+u1;
tol(2)=-(Kp1(2,2)+Kp2(2,2)/(alfa2+abs(e2)))*e2-(Kv1(2,2)+Kv2(2,2)/(beta2+abs(de2)))*de2+fy(2)+u2;

sys(1)=tol(1);
sys(2)=tol(2);