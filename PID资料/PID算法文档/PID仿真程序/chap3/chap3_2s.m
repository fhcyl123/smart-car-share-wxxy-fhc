function [sys,x0,str,ts]=exp_pidf(t,x,u,flag,T,kp,ki,kd,MTab)
switch flag,
case 0           % initializations
    [sys,x0,str,ts] = mdlInitializeSizes(T);
case 2           % discrete states updates
    sys = mdlUpdates(x,u,T);
case 3           % computation of control signal
    sys = mdlOutputs(t,x,u,kp,ki,kd,MTab);
case {1, 4, 9}   % unused flag values
    sys = [];
otherwise        % error handling
    error(['Unhandled flag = ',num2str(flag)]);
end;

%==============================================================
% when flag=0, perform system initialization
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes(T)
sizes = simsizes;        % read default control variables
sizes.NumContStates = 0; % no continuous states
sizes.NumDiscStates = 3; % 3 states and assume they are the P/I/D components
sizes.NumOutputs = 2;    % 2 output variables: control u(t) and state x(3)
sizes.NumInputs = 4;     % 4 input signals
sizes.DirFeedthrough = 1;% input reflected directly in output
sizes.NumSampleTimes = 1;% single sampling period
sys = simsizes(sizes);   % 
x0 = [0; 0; 0];          % zero initial states
str = []; 
ts = [-1 0];             % sampling period
%==============================================================
% when flag=2, updates the discrete states
%==============================================================
function sys = mdlUpdates(x,u,T)
sys=[ u(1); 
      x(2)+u(1)*T;
      (u(1)-u(2))/T];
%x(1):error value
%x(2):error integrate 
%x(3):error difference 
%==============================================================
% when flag=3, computates the output signals
%==============================================================
function sys = mdlOutputs(t,x,u,kp,ki,kd,MTab)

i=find(abs(x(1))>MTab(:,1));        %Rule1
if length(i)>0
    sys=MTab(i(1),2);
else
    sys=[kp, ki, kd]*x;
end

if x(1)*x(3)>0|(abs(x(3))<eps)       %Rule2
   if abs(x(1))>=0.05
      sys=u(3)+2*kp*x(1);
   else
      sys=u(3)+0.4*kp*x(1);
   end
end

if x(1)*x(3)<0 & x(3)*u(4)<0         %Rule4
   if abs(x(1))>=0.05
      sys=u(3)+2*kp*u(2);
   else
      sys=u(3)+0.6*kp*u(2);
   end
end

if abs(x(1))<=0.001                  %Rule5:Integration separation PI control
   sys=0.5*x(1)+0.010*x(2);
end

sys=[sys; x(3)];