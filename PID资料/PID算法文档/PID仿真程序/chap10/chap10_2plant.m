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
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; % At least one sample time is needed
sys = simsizes(sizes);
x0  = [0;0];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(t,x,u)   %Time-varying model

g=9.8;
m=1;
l=0.25;
d=2.0;
I=4/3*m*l^2;

delta0=0.20;
d=2.0;
delta1=0.80;
dp=d+delta1;      %Prediction value

tol=u;

sys(1)=x(2);
sys(2)=1/I*(-dp*x(2)-delta0*x(1)-m*g*l*cos(x(1))+tol);

function sys=mdlOutputs(t,x,u)
sys(1)=x(1);
sys(2)=x(2);