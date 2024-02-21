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
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0; % At least one sample time is needed
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)

g=9.8;
m=1;
l=0.25;
d=2.0;
a=200;b=150;
I=4/3*m*l^2;

A=1.0;F=1.0;
r=u(1);
x1=u(2);
x2=u(3);

dr=A*F*2*pi*cos(F*2*pi*t);
ddr=-A*(F*2*pi)^2*sin(2*pi*t);

e=x1-r;
de=x2-dr;

tol=(d-a*I)*de-b*I*e+I*ddr+d*dr+m*g*l*cos(x1);
sys(1)=tol;