%Nonlinear PID Control for a servo system
clear all;
close all;

ts=0.001;
J=1/133;
q=25/133;
sys=tf(1,[J,q,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

r_1=0;r_2=0;
u_1=0;u_2=0;
y_1=0;y_2=0;
error_1=0;
ei=0;
for k=1:1:1000
time(k)=k*ts;

S=1;
if S==1      %Step Signal
   rin(k)=1.0;
elseif S==2  %Sine Signal
   rin(k)=1.0*sin(1*2*pi*k*ts);
end

yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
error(k)=rin(k)-yout(k);  
derror(k)=(error(k)-error_1)/ts; 

ap=22;bp=8.0;cp=0.8;
kp(k)=ap+bp*(1-sech(cp*error(k)));

ad=0.5;bd=2.5;cd=6.5;dd=0.30;
kd(k)=ad+bd/(1+cd*exp(dd*error(k)));

ai=1;ci=1;
ki(k)=ai*sech(ci*error(k));

ei=ei+error(k)*ts;
u(k)=kp(k)*error(k)+kd(k)*derror(k)+ki(k)*ei;

%Update Parameters
r_2=r_1;r_1=rin(k);
u_2=u_1;u_1=u(k);
y_2=y_1;y_1=yout(k);
error_1=error(k);
end
figure(1);
plot(time,rin,'k',time,yout,'k');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,rin-yout,'k');ylabel('error');
xlabel('time(s)');ylabel('error');
figure(3);
plot(time,derror,'k');
xlabel('time(s)');ylabel('derror');

M=1;
if M==1
   figure(4);
   subplot(311);
	plot(error,kp,'k');xlabel('error');ylabel('kp');
   subplot(312);
	plot(error,kd,'k');xlabel('error');ylabel('kd');
	ad+bd/(1+cd)
   subplot(313);
	plot(error,ki,'k');xlabel('error');ylabel('ki');
elseif M==2
   figure(5);
   subplot(311);
	plot(time,kp,'k');xlabel('time(s)');ylabel('kp');
   subplot(312);
	plot(time,kd,'k');xlabel('time(s)');ylabel('kd');
   subplot(313);
	plot(time,ki,'k');xlabel('time(s)');ylabel('ki');
end