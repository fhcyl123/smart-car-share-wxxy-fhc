function [sys,x0,str,ts]=mm_model_nl(t,x,u,flag,T)
switch flag,
case 0 % Initialization
    [sys,x0,str,ts] = mdlInitializeSizes(T);
case 3 % evaluation of outputs
    sys = mdlOutputs(u);
case {1, 2, 4, 9} % undefined flag values
    sys = [];
otherwise % error handling
    error(['Unhandled flag = ',num2str(flag)]);
end;

%==============================================================
% when flag==0, initialization processed for the system
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes(T)
sizes = simsizes; % read the default templates for the system variables
sizes.NumContStates = 0; % no continuous states
sizes.NumDiscStates = 0; % 6 discrete states, [x1-x3]
sizes.NumOutputs = 2; % control variable u(k) and PID parameters
sizes.NumInputs = 6; % 7 input signals
sizes.DirFeedthrough = 1; % inputs are needed in output evaluation
sizes.NumSampleTimes = 1; % single sampling period
sys = simsizes(sizes); % setting of system variables
x0 = []; % zero states, and 0.1 for initial weights
str = []; 
ts = [T 0]; % T is the sampling period for the system

function sys = mdlOutputs(u)
sys=[(0.8*u(1)+u(3)+0.2*u(6))/(1+u(1))^2;
     (0.9*u(2)+0.3*u(4)+u(5))/(1+u(2))^2];