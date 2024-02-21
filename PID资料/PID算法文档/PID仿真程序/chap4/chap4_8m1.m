function [u]=chap4_8m1(u1,u2,u3)
global s
persistent errori error_1

ts=0.001;
if u3==0
   errori=0;
   error_1=0;
end   

s=2;            %Selecting Signal Type
if s==1         %Step Signal
   kp=0.4;
   ki=0.0;
   kd=0.28;
elseif s==2     %Square Wave Signal
   kp=0;
   ki=0;
   kd=0.28;
end
error=u2;
errord=(error-error_1)/ts;
errori=errori+error*ts;

u=kp*error+kd*errord+ki*errori;
error_1=error;