%PID Controller for coupling plant
clear all;
close all;

u1_1=0.0;u1_2=0.0;u1_3=0.0;u1_4=0.0;
u2_1=0.0;u2_2=0.0;u2_3=0.0;u2_4=0.0;

y1_1=0;y2_1=0;

x1=[0;0];x2=[0;0];x3=[0;0];

kp=0.020;
ki=0.050;
kd=0.0001;

error_1=[0;0];
ts=1;
for k=1:1:1500
time(k)=k*ts;

%Step Signal
%R=[1;0];
R=[0;1];

%PID Decouple Controller
u1(k)=kp*x1(1)+kd*x2(1)+ki*x3(1);   
u2(k)=kp*x1(2)+kd*x2(2)+ki*x3(2);   
u=[u1(k),u2(k)];

if u1(k)>=10
   u1(k)=10;
end
if u2(k)>=10
   u2(k)=10;
end
if u1(k)<=-10
   u1(k)=-10;
end
if u2(k)<=-10
   u2(k)=-10;
end

%Coupling Plant
yout1(k)=1.0/(1+y1_1)^2*(0.8*y1_1+u1_2+0.2*u2_3);
yout2(k)=1.0/(1+y2_1)^2*(0.9*y2_1+0.3*u1_3+u2_2);

error1(k)=R(1)-yout1(k);
error2(k)=R(2)-yout2(k);
error=[error1(k);error2(k)];

%-----------Return of PID parameters------------%
u1_4=u1_3;u1_3=u1_2;u1_2=u1_1;u1_1=u(1);

u2_4=u2_3;u2_3=u2_2;u2_2=u2_1;u2_1=u(2);

y1_1=yout1(k);y2_1=yout2(k);

x1=error;                %Calculating P
x2=(error-error_1)/ts;   %Calculating D
x3=x3+error*ts;          %Calculating I

error_1=error;
end
figure(1);
plot(time,R(1),'k',time,yout1,'k');
hold on;
plot(time,R(2),'k',time,yout2,'k');
xlabel('time(s)');ylabel('rin,yout');