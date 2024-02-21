% S-function for continuous state equation
function [sys,x0,str,ts]=s_function(t,x,u,flag)

switch flag,
%Initialization
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
%Outputs
  case 3,
    sys=mdlOutputs(t,x,u);
%Unhandled flags
  case {1, 2, 4, 9 }
    sys = [];
%Unexpected flags
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

%mdlInitializeSizes
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;

sys=simsizes(sizes);
x0=[];
str=[];
ts=[];

%mdlOutputs
function sys=mdlOutputs(t,x,u)
global K
S=1;
if S==1
   sys=-K*[u(1)-u(7),u(2),u(3),u(4),u(5),u(6)]';   %LQR
elseif S==2   
   sys=50*(u(7)-u(1))+10*(0-u(2))+10*(0-u(3))+10*(0-u(4))-10*(0-u(5))+10*(0-u(6));%PID
end