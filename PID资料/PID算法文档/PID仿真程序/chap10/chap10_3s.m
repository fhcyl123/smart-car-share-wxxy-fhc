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
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys=mdlOutputs(t,x,u)
g=9.8;
m=1;
l=0.25;
d=2.0;
I=4/3*m*l^2;

AA=2.0;
FF=0.10;
r=u(1);
dr=AA*FF*2*pi*cos(FF*2*pi*t);
ddr=-AA*(FF*2*pi)^2*sin(FF*2*pi*t);

x1=u(2);
x2=u(3);

e=x1-r;
de=x2-dr;

a=20;b=25;
A=[0 1;-b -a];
B=[0;1];

Q=[10 0;
   0 1];
P=lyap(A',Q);
eig(P);

delta0=0.2;
delta1=0.2;

w=-(delta1*dr+delta0*r)/I;
%df=w-delta1/I*de-delta0/I*e;

gama=9;
beta=0.3;

rou=abs(w)+0.10;
rou1=rou+1/I*(abs(de)+abs(e));

xe=[e;de];
v=-xe'*P*B*rou1^2/(abs(xe'*P*B)*rou1+gama*exp(-beta*t));
%v=0;

d1=delta1+d;
tol=(d-a*I)*de-b*I*e+I*ddr+d*dr+m*g*l*cos(x1)+I*v;

sys(1)=tol;