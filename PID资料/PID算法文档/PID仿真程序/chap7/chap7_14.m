%Discrete PID control with Anti-windup
clear all;
close all;

xk=zeros(2,1);
e_1=0;
ei=0;
u_1=0;

alfa=1.0;

kp=0.10;
ki=0.01;
kd=0.01;

umin=0;
umax=10;

ts=1;
for k=1:1:300
time(k)=k*ts;

rin(k)=1500;
  
para=u_1;
tSpan=[0 ts];
[tt,xx]=ode45('chap7_14f',tSpan,xk,[],para);
xk=xx(length(xx),:);
yout(k)=xk(1); 

e(k)=rin(k)-yout(k);
de(k)=(e(k)-e_1)/ts; 

un(k)=kp*e(k)+ki*ei+kd*de(k);

us(k)=un(k);
if us(k)>=umax
   us(k)=umax;
end
if us(k)<=umin
   us(k)=umin;
end

es(k)=us(k)-un(k);

M=1;
switch M
case 1        %VSPID
	ua(k)=(umax+umin)/2;
	if un(k)~=us(k)&e(k)*(un(k)-ua(k))>0
	   ef(k)=-alfa*(un(k)-us(k))/ki;   
	else
	   ef(k)=e(k);
   end
case 2        %No Anti-windup
   ef(k)=e(k);
end
ei=ei+ef(k)*ts;

u_1=us(k);
e_1=e(k);
end
figure(1);
plot(time,rin,'r',time,yout,'b');
xlabel('time(s)'),ylabel('rin,yout');
figure(2);
plot(time,us,'r');
xlabel('time(s)'),ylabel('control input');