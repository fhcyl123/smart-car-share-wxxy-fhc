%PID control based on Hopfield
clear all;
close all;

ts=0.001;
TimeSet=[0:ts:1];

para=[];

[t,x]=ode45('chap4_9eq',TimeSet,[0 0 0 0],[],para);
vm=x(:,1);
v=x(:,2);
F=x(:,3);
G=x(:,4);

vd=30;
I=-F.*v+G*vd;

kp=F;
kf=G-kp;

figure(1);
plot(t,vm,'r',t,v,'b',t,vd,'k');
xlabel('time(s)');ylabel('speed tracking');

figure(2);
plot(t,I,'r');
xlabel('time(s)');ylabel('control input');

figure(3);
plot(t,kp,'r');
xlabel('time(s)');ylabel('kp');

figure(4);
plot(t,kf,'r');
xlabel('time(s)');ylabel('kf');