%Adaptive Robust Control Based on PD Term
clear all;
close all;
global S

ts=0.001;
TimeSet=[0:ts:60];

a0=30;
a1=20;
b=50;

Am=[0,1;-a0,-a1];
eig(Am)
%Q=[20,0;0,20];
Q=[20,10;10,20];

P=lyap(Am',Q);
p12=P(1,2);
p22=P(2,2);
para=[a1,a0,b,p12,p22];

[t,yout]=ode45('chap7_16eq',TimeSet,[0 0 0 0 0 0 0],[],para);
k0=yout(:,5);
k1=yout(:,6);
k2=yout(:,7);

switch S
case 1
   r=1.0*sign(sin(0.05*t*2*pi));    %Square Signal
case 2
   r=1.0*sin(1.0*t*2*pi);           %Sin Signal
end
u=k0.*r+k1.*yout(:,3)+k2.*yout(:,4);

figure(1);
plot(t,yout(:,1),'k',t,yout(:,3),'k');
xlabel('Time(s)');ylabel('Position tracking');

figure(2);
plot(t,yout(:,1)-yout(:,3),'k');
xlabel('Time(s)');ylabel('Position tracking error');

figure(3);
plot(t,u,'k');
xlabel('Time(s)');ylabel('Control input');

figure(4);
subplot(3,1,1);
plot(t,k0,'k');
xlabel('Time(s)');ylabel('k0');
subplot(3,1,2);
plot(t,k1,'k');
xlabel('Time(s)');ylabel('k1');
subplot(3,1,3);
plot(t,k2,'k');
xlabel('Time(s)');ylabel('k2');