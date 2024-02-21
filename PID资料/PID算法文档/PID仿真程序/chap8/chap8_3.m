%Discreted PID with grey model prediction
clear all;
close all;
ts=0.001;

n=2;
N=n+3;

a=25;b=133;
J=1/133;
q=25/133;
sys=tf(1,[J,q,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

A1=[0,1;0,-a];
b1=[0;b];
C1=[1,0];
D1=0;
[A,b,C,D]=c2dm(A1,b1,C1,D1,ts,'z');
A=A;
b=-b;

x_0=[0;1];
x_1=x_0;

%Uncertain Parameters
V=[0.5 0.5];d=0.5;
%Initial Value
x_1=[1;1];

%Grey prediction
for k=1:1:N
	time(k)=k*ts;
	x1(k)=x_1(1);
	x2(k)=x_1(2);

	D(k)=V*x_1+d;
	   
	kp=2.0;
	up(k)=kp*x1(k);
	u(k)=up(k);   
   
	D=V*x_1+d;
	x=A*x_1+b*u(k)+b*D;
	x_1=x;
end
   xx1(1)=x1(2);
   xx2(1)=x2(2);
	BB=[xx1(1) xx2(1) 1];

for i=2:1:N-2
   xx1(i)=xx1(i-1)+x1(i+1);
   xx2(i)=xx2(i-1)+x2(i+1);
   BB=[BB;xx1(i) xx2(i) i];
end

for i=1:1:N-1
   D(i)=1/b*([x1(i+1);x2(i+1)]-A*[x1(i);x2(i)]-b*up(i));
end
   D1(1)=D(2);
for i=2:1:N-2
   D1(i)=D1(i-1)+D(i+1);
end

%abs(det(BB'*BB));
V1=inv(BB'*BB)*BB'*D1';
Vp=V1'

%Grey PID control
x_1=x_0;
N1=2000;
for k=1:1:N1
time(k)=k*ts;
x1(k)=x_1(1);
x2(k)=x_1(2);

D(k)=V*x_1+d;
   
%Control law
M=1;
if M==1      %No Grey Compensation
   uc(k)=0;   
elseif M==2  %Grey Compensation
   uc(k)=-(Vp(1)*x_1(1)+Vp(2)*x_1(2)+Vp(3));   
end
up(k)=kp*x1(k);
u(k)=up(k)+uc(k);
   
D=V*x_1+d;
x=A*x_1+b*u(k)+b*D;
x_1=x;

end
figure(1);
plot(time,x1);
xlabel('time(s)');ylabel('error');
figure(2);
plot(time,x2);
xlabel('time(s)');ylabel('derror');
figure(3);
plot(time,u);
xlabel('time(s)');ylabel('u');