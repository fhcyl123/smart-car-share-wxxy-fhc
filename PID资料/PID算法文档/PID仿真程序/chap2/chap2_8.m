%PID Controler (2001/9/6) 
close all;

ts=0.25;
sys=tf(1,[10,2,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

u_1=0;u_2=0;
y_1=0;y_2=0;

x=[0,0,0]';

error_1=0;

for k=1:1:1000
time(k)=k*ts;

%rin(k)=1.0;
rin(k)=0.5*sin(0.025*2*pi*k*ts);

%Linear model
yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
error(k)=rin(k)-yout(k);

x(1)=error(k);                % Calculating P
x(2)=(error(k)-error_1)/ts;   % Calculating D
x(3)=x(3)+error(k)*ts;        % Calculating I

M=2;
switch M
   case 1       %Using PID
	u(k)=kp*x(1)+kd*x(2)+ki*x(3);
   case 2       %No PID
   u(k)=error(k);
end

u_2=u_1;
u_1=u(k);
 
y_2=y_1;
y_1=yout(k);
   
error_1=error(k);
end
figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('(rin,yout)');

figure(2);
plot(time,rin-yout,'r');
xlabel('time(s)');ylabel('error');