%Single Neural Net PID Decouple Controller based on Hebb Learning 
%Algorithm to adjust kp,ki,kd
function [sys,x0,str,ts]=exp_pidf(t,x,u,flag)
switch flag,
case 0           % initializations
    [sys,x0,str,ts] = mdlInitializeSizes;
case 2           % discrete states updates
    sys = mdlUpdates(x,u);
case 3           % computation of control signal
    sys=mdlOutputs(t,x,u);
case {1, 4, 9}   % unused flag values
    sys = [];
otherwise        % error handling
    error(['Unhandled flag = ',num2str(flag)]);
end;

%==============================================================
% when flag=0, perform system initialization
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;        % read default control variables
sizes.NumContStates = 0; % no continuous states
sizes.NumDiscStates = 3; % 3 states and assume they are the P/I/D components
sizes.NumOutputs = 1;    % 2 output variables: control u(t) and state x(3)
sizes.NumInputs = 3;     % 4 input signals
sizes.DirFeedthrough = 1;% input reflected directly in output
sizes.NumSampleTimes = 1;% single sampling period
sys = simsizes(sizes);   % 
x0 = [0; 0; 0];          % zero initial states
str = []; 
ts = [-1 0];             % sampling period
%==============================================================
% when flag=2, updates the discrete states
%==============================================================
function sys = mdlUpdates(x,u)
T=1;  
sys=[ u(1); 
      x(2)+u(1)*T;
      (u(1)-u(2))/T];

%==============================================================
% when flag=3, computates the output signals
%==============================================================
function sys = mdlOutputs(t,x,u)
persistent wkp1_1 wki1_1 wkd1_1 u1_1
xiteP=0.60;
xiteI=0.60;
xiteD=0.60;

if t==0  %Initilizing kp,ki and kd
    wkp1_1=0.3;
    wki1_1=0.3;
    wkd1_1=0.3;
    u1_1=0;
end

%Adjusting NNC Weight Value by adopting hebb learning algorithm
   wkp1=wkp1_1+xiteP*x(1)*u1_1*x(1);  %P
   wki1=wki1_1+xiteI*x(1)*u1_1*x(2);  %I 
   wkd1=wkd1_1+xiteD*x(1)*u1_1*x(3);  %D

   wadd1=abs(wkp1)+abs(wki1)+abs(wkd1);
   w111=wkp1/wadd1;
   w122=wki1/wadd1;
   w133=wkd1/wadd1;
   w1=[w111,w122,w133];
   k1=0.20;
   u1=k1*w1*x;
   
   wkp1_1=wkp1;
   wkd1_1=wkd1;
   wki1_1=wki1;

   u1_1=u1;

   sys=u1;