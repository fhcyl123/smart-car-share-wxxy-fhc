%PID Controller
clear all;
close all;

ts=0.001;
sys=tf(5.235e005,[1,87.35,1.047e004,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

u_1=0.0;u_2=0.0;u_3=0.0;
r_1=rand;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
error_1=0;

for k=1:1:3000
time(k)=k*ts;

kp=1.0;ki=2.0;kd=0.01;
   
S=1;   
if S==1   %Triangle Signal
   if mod(time(k),2)<1     
	   rin(k)=mod(time(k),1);
	else
	   rin(k)=1-mod(time(k),1);
	end
	   rin(k)=rin(k)-0.5;
end
if S==2   %Sawtooth Signal
   rin(k)=mod(time(k),1.0);
end
if S==3   %Random Signal
	rin(k)=rand;  
	vr(k)=(rin(k)-r_1)/ts;  %Max speed is 5.0
	while abs(vr(k))>=5.0
   	rin(k)=rand;  
		vr(k)=abs((rin(k)-r_1)/ts);
	end
end

u(k)=kp*x(1)+kd*x(2)+ki*x(3);   %PID Controller

%Restricting the output of controller
if u(k)>=10
   u(k)=10;
end
if u(k)<=-10
   u(k)=-10;
end

%Linear model
yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k)=rin(k)-yout(k);

r_1=rin(k);

u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
   
x(1)=error(k);                %Calculating P
x(2)=(error(k)-error_1)/ts;   %Calculating D
x(3)=x(3)+error(k)*ts;        %Calculating I
xi(k)=x(3);

error_1=error(k);
D=0;
if D==1  %Dynamic Simulation Display
	plot(time,rin,'b',time,yout,'r');
	pause(0.00000000000000000);
end
end
plot(time,rin,'r',time,yout,'b');
xlabel('time(s)');ylabel('rin,yout');