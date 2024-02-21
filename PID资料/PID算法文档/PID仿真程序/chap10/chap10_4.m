%PD Control for Robotic Manipulator (2003/04/15)
clear all;
close all;
global F

x0=[0,0,0,0];

ts=0.001;
T=5.0;
TimeSet=[0:ts:T];
para=[];
[t,y]=ode45('chap10_4eq',TimeSet,x0,[],para);

switch F
case 1
   r1=pi/6;r2=pi/6;
   dr1=0;dr2=0;
case 2
   r1=sin(3*t);r2=cos(3*t);
   dr1=3*cos(3*t);dr2=-3*sin(3*t);
case 3
   r1=sign(sin(3*t));r2=sign(sin(3*t));
   dr1=0;dr2=0;
end

figure(1);
plot(t,r1,'r',t,y(:,1),'b');
xlabel('time(s)');ylabel('r1,x(1)');
figure(2);
plot(t,dr1,'r',t,y(:,2),'b');
xlabel('time(s)');ylabel('dr1,x(2)');

figure(3);
plot(t,r2,'r',t,y(:,3),'b');
xlabel('time(s)');ylabel('r2,x(3)');
figure(4);
plot(t,dr2,'r',t,y(:,4),'b');
xlabel('time(s)');ylabel('dr2,x(2)');