function [sys,x0,str,ts] = spacemodel(t,x,u,flag)

switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [0;0];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(t,x,u)
R1=u(1);
dr1=-pi*sin(pi*t);
ddr1=-pi^2*cos(pi*t);
R2=u(2);
dr2=pi*cos(pi*t);
ddr2=-pi^2*sin(pi*t);

x1=u(3);
x2=u(4);
x3=u(5);
x4=u(6);

e1=x1-R1;
e2=x3-R2;
de1=x2-dr1;
de2=x4-dr2;

f=max([1,norm([e1,e2]),norm([de1,de2])]);

gama=1.5;
y1=de1+gama*e1;
y2=de2+gama*e2;

r1=250;r2=250;

sys(1)=r1*f*norm([y1,y2]);
sys(2)=-r2*x(2);

function sys=mdlOutputs(t,x,u)
R1=cos(pi*t);
dr1=-pi*sin(pi*t);
ddr1=-pi^2*cos(pi*t);
R2=sin(pi*t);
dr2=pi*cos(pi*t);
ddr2=-pi^2*sin(pi*t);

x1=u(3);
x2=u(4);
x3=u(5);
x4=u(6);

e1=x1-R1;
e2=x3-R2;
de1=x2-dr1;
de2=x4-dr2;

v=13.33;
q1=8.98;
q2=8.75;
g=9.8;

M=[v+q1+2*q2*cos(x3) q1+q2*cos(x3);
   q1+q2*cos(x3) q1];

C=[-q2*x4*sin(x3) -q2*(x2+x4)*sin(x3);
    q2*x2*sin(x3)  0];

G=[15*g*cos(x1)+8.75*g*cos(x1+x3);
   8.75*g*cos(x1+x3)];

f=max([1,norm([e1,e2]),norm([de1,de2])]);

gama=1.5;
y1=de1+gama*e1;
y2=de2+gama*e2;
u1=-(x(1)*f)^2*y1/(x(1)*f*norm([y1 y2])+x(2)^2+0.0000001);
u2=-(x(1)*f)^2*y2/(x(1)*f*norm([y1 y2])+x(2)^2+0.0000001);

dr=[dr1;dr2]-gama*[e1;e2];
ddr=[ddr1;ddr2]-gama*[de1;de2];

fy=M*ddr+C*dr+G;

Kp1=[180,0;0,190];
Kp2=[150,0;0,150];
Kv1=[180,0;0,180];
Kv2=[150,0;0,150];

alfa1=1;alfa2=1;
beta1=1;beta2=1;

tol(1)=-(Kp1(1,1)+Kp2(1,1)/(alfa1+abs(e1)))*e1-(Kv1(1,1)+Kv2(1,1)/(beta1+abs(de1)))*de1+fy(1)+u1;
tol(2)=-(Kp1(2,2)+Kp2(2,2)/(alfa2+abs(e2)))*e2-(Kv1(2,2)+Kv2(2,2)/(beta2+abs(de2)))*de2+fy(2)+u2;

sys(1)=tol(1);
sys(2)=tol(2);
sys(3)=x(1);