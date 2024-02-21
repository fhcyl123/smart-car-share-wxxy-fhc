%PID Control based on Disturbance Observer
clear all;
close all;

Jp=0.0075;bp=0.1880;
Jn=Jp;bn=bp;

ts=0.001;

Gp=tf([1],[Jp,bp,0]);   %Plant
Gpz=c2d(Gp,ts,'z');
[num,den]=tfdata(Gpz,'v');

Gn=tf([1],[Jn,bn,0]);   %Nominal model
Gnz=c2d(Gn,ts,'z');
[num1,den1]=tfdata(Gnz,'v');

tol=0.0065;
Q=tf([3*tol,1],[tol^3,3*tol^2,3*tol,1]);   %Low Pass Filter
Qz=c2d(Q,ts,'tustin');
[numq,denq]=tfdata(Qz,'v');

uu_1=0;
u2_1=0;
uo_1=0;uo_2=0;uo_3=0;
u3_1=0;u3_2=0;u3_3=0;

u_1=0.0;u_2=0.0;
y_1=0;y_2=0;

x=[0,0,0]';
error_1=0;

for k=1:1:1000
time(k)=k*ts;
   
rin(k)=0.5*sin(3*2*pi*k*ts);  % Tracing Sine high frequency Signal

%Linear model
yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;

n(k)=0.001*rands(1);          % Measure noise
yout(k)=yout(k)+n(k);         % Disturbance n(k)

error(k)=rin(k)-yout(k);

x(1)=error(k);                % Calculating P
x(2)=(error(k)-error_1)/ts;   % Calculating D
x(3)=x(3)+error(k)*ts;        % Calculating I

kp=50;ki=0;kd=15.0;           % Tracing Sine velocity

c(k)=kp*x(1)+kd*x(2)+ki*x(3); % PID Controller

u1(k)=uu_1;  %Mn=1, one time delay

u2(k)=1/num1(2)*(-num1(3)*u2_1+yout(k)+den1(2)*y_1+den1(3)*y_2);

u3(k)=u2(k)-u1(k);

uo(k)=-denq(2)*uo_1-denq(3)*uo_2-denq(4)*uo_3+numq(1)*u3(k)+numq(2)*u3_1+numq(3)*u3_2+numq(4)*u3_3;

Q=1;
if Q==0     %Not using Q(s)
	uo(k)=u3(k);  
end

M=1;
if M==1     %Using Observer 
   uu(k)=c(k)-uo(k);
end
if M==2     %No Observer
   uu(k)=c(k);
end

d(k)=50*sin(5*2*pi*k*ts);
u(k)=uu(k)+d(k);   % Disturbance d(k)

if u(k)>=110       % Restricting the output of controller
   u(k)=110;
end
if u(k)<=-110
   u(k)=-110;
end

uu_1=uu(k);
u2_1=u2(k);

uo_3=uo_2;uo_2=uo_1;uo_1=uo(k);
u3_3=u3_2;u3_2=u3_1;u3_1=u3(k);

u_2=u_1;u_1=u(k);
y_2=y_1;y_1=yout(k);
   
error_1=error(k);
end
figure(1);
subplot(211);
plot(time,u3,'k');
xlabel('time(s)');ylabel('u3');
subplot(212);
plot(time,uo,'k');
xlabel('time(s)');ylabel('uo');
figure(2);
plot(time,d,'k',time,uo,'k');
xlabel('time(s)');ylabel('d,uo');
figure(3);
plot(time,rin,'k',time,yout,'k');
xlabel('time(s)');ylabel('rin,yout');