%S-function for continuous state equation
function [sys,x0,str,ts]=s_function(t,x,u,flag)

switch flag,
%Initialization
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
%Outputs
  case 3,
    sys=mdlOutputs(t,x,u);
%Unhandled flags
  case {2, 4, 9 }
    sys = [];
%Unexpected flags
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

%mdlInitializeSizes
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;

sys=simsizes(sizes);
x0=[0.6;0.3;0.5;0.5];
str=[];
ts=[];

function sys=mdlDerivatives(t,x,u)
R1=cos(pi*t);
dr1=-pi*sin(pi*t);
ddr1=-pi^2*cos(pi*t);
R2=sin(pi*t);
dr2=pi*cos(pi*t);
ddr2=-pi^2*sin(pi*t);

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
w=[d1+d2*norm([e1,e2])+d3*norm([de1,de2])];

tol(1)=u(1);
tol(2)=u(2);

S=inv(M)*(tol'-C*[x(2);x(4)]-G-w);

sys(1)=x(2);
sys(2)=S(1);
sys(3)=x(4);
sys(4)=S(2);
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);
sys(4)=x(4);