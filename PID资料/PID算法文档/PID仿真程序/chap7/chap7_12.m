%Single Link Inverted Pendulum Control
clear all;
close all;
global A B C D

%Single Link Inverted Pendulum Parameters
g=9.8;
M=1.0;
m=0.1;
L=0.5;
Fc=0.0005;
Fp=0.000002;

I=1/12*m*L^2;  
l=1/2*L;
t1=m*(M+m)*g*l/[(M+m)*I+M*m*l^2];
t2=-m^2*g*l^2/[(m+M)*I+M*m*l^2];
t3=-m*l/[(M+m)*I+M*m*l^2];
t4=(I+m*l^2)/[(m+M)*I+M*m*l^2];

A=[0,1,0,0;
   t1,0,0,0;
   0,0,0,1;
   t2,0,0,0];
B=[0;t3;0;t4];
C=[1,0,0,0;
   0,0,1,0];
D=[0;0];

Q=[100,0,0,0;   %100,10,1,1 express importance of theta,dtheta,x,dx
   0,10,0,0;
   0,0,1,0;
   0,0,0,1];
R=[0.1];
K=LQR(A,B,Q,R); %LQR Gain    

e1_1=0;e2_1=0;e3_1=0;e4_1=0;
u_1=0;
xk=[-10/57.3,0,0.20,0];   %Initial state

ts=0.02;
for k=1:1:1000
time(k)=k*ts;
Tspan=[0 ts];

para=u_1;
[t,x]=ode45('chap7_11f',Tspan,xk,[],para);
xk=x(length(x),:);

r1(k)=0.0;    %Pendulum Angle
r2(k)=0.0;    %Pendulum Angle Rate
r3(k)=0.0;    %Car Position
r4(k)=0.0;    %Car Position Rate

x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
x4(k)=xk(4);

e1(k)=r1(k)-x1(k);
e2(k)=r2(k)-x2(k);
e3(k)=r3(k)-x3(k);
e4(k)=r4(k)-x4(k);

S=1;
if S==1      %LQR
	u(k)=K(1)*e1(k)+K(2)*e2(k)+K(3)*e3(k)+K(4)*e4(k);
elseif S==2  %PD
   de1(k)=e1(k)-e1_1;
   u1(k)=-50*e1(k)-10*de1(k);
   de2(k)=e2(k)-e2_1;
   u2(k)=-10*e2(k)-10*de2(k);
   de3(k)=e3(k)-e3_1;
   u3(k)=-10*e3(k)-10*de3(k);
   de4(k)=e4(k)-e4_1;
   u4(k)=-10*e4(k)-10*de4(k);
   u(k)=u1(k)+u2(k)+u3(k)+u4(k);
end

if u(k)>=10
   u(k)=10;
elseif u(k)<=-10
  	u(k)=-10;
end
	e1_1=e1(k);
	e2_1=e2(k);
	e3_1=e3(k);
	e4_1=e4(k);
	u_1=u(k);
end
figure(1);
subplot(411);
plot(time,r1,'k',time,x1,'k');      %Pendulum Angle
xlabel('time(s)');ylabel('Angle');
subplot(412);
plot(time,r2,'k',time,x2,'k');      %Pendulum Angle Rate
xlabel('time(s)');ylabel('Angle rate');
subplot(413);
plot(time,r3,'k',time,x3,'k');      %Car Position
xlabel('time(s)');ylabel('Cart position');
subplot(414);
plot(time,r4,'k',time,x4,'k');      %Car Position Rate
xlabel('time(s)');ylabel('Cart rate');
figure(5);
plot(time,u,'k');                   %Force F change
xlabel('time(s)');ylabel('Force');