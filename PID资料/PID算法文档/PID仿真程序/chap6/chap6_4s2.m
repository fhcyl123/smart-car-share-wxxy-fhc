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
persistent wkp2_1 wki2_1 wkd2_1 u2_1
xiteP=0.60;
xiteI=0.60;
xiteD=0.60;

if t==0  %Initilizing kp,ki and kd
    wkp2_1=0.3;
    wki2_1=0.3;
    wkd2_1=0.3;
    u2_1=0;
end

%Adjusting NNC Weight Value by adopting hebb learning algorithm
   wkp2=wkp2_1+xiteP*x(1)*u2_1*x(1);  %P
   wki2=wki2_1+xiteI*x(1)*u2_1*x(2);  %I 
   wkd2=wkd2_1+xiteD*x(1)*u2_1*x(3);  %D

   wadd2=abs(wkp2)+abs(wki2)+abs(wkd2);
   w211=wkp2/wadd2;
   w222=wki2/wadd2;
   w233=wkd2/wadd2;
   w2=[w211,w222,w233];
   k2=0.20;
   u2=k2*w2*x;
   
   wkp2_1=wkp2;
   wkd2_1=wkd2;
   wki2_1=wki2;

   u2_1=u2;

   sys=u2;