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
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; % At least one sample time is needed
sys = simsizes(sizes);
x0  = [0;0;0];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(t,x,u)   %Lugre model
global J rou0 rou1 af

Fc=0.28;
Fs=0.34;
vs=0.01;

%Ref:pid_fm_eq.m
g=Fc+(Fs-Fc)*exp(-(x(2)/vs)^2)+af*x(2);
sys(3)=x(2)-(rou0*abs(x(2))/g)*x(3);
F=rou0*x(3)+rou1*sys(3)+af*x(2);
sys(1)=x(2);
sys(2)=1/J*(u-F);  %Important!

function sys=mdlOutputs(t,x,u)
sys(1)=x(1);   %Angle
sys(2)=x(2);   %Angle speed
sys(3)=x(3);   %z