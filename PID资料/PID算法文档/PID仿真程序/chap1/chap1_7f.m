function [u]=pidsimf(u1,u2)
persistent pidmat errori error_1

if u1==0
   errori=0
   error_1=0
end   

ts=0.001;
kp=1.5;
ki=2.0;
kd=0.05;

error=u2;
errord=(error-error_1)/ts;
errori=errori+error*ts;

u=kp*error+kd*errord+ki*errori;
error_1=error;