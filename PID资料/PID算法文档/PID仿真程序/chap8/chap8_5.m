%Discrete PID Control with grey model prediction
clear all;
close all;

ts=0.001;
n=2;

AA=0.5;
F=3.0;
N1=1000;

N=n+3;
w=2*pi*F;

%System model
A1=[0 1;0 -25];
B1=[0;133];
C1=[1 0];
D1=[0];
[A,B,C,D]=c2dm(A1,B1,C1,D1,ts,'z');

V=[5 -5];d=5;
x_1=[0;0];
for G=1:2
   for k=1:1:N
   time(k)=k*ts;
   
   r(k)=AA*(sin(w*k*ts));
   dr(k)=AA*w*cos(w*k*ts);
   
   x1(k)=x_1(1);
   x2(k)=x_1(2);
      
%Control law
if G==1         %For Grey Prediction
   uc(k)=0;     
elseif G==2     %For Grey PID Control
   M=2;
   if M==1      %No Grey Compensation
      uc(k)=0; 
   elseif M==2  %Grey Compensation
      uc(k)=-(Vp(1)*x_1(1)+Vp(2)*x_1(2)+Vp(3));
   end
end

kp=80;kd=10;
e(k)=r(k)-x1(k);
de(k)=dr(k)-x2(k);

up(k)=kp*e(k)+kd*de(k);
u(k)=up(k)+uc(k);
   
%Plant   
   DD=V*x_1+d;
   x=A*x_1+B*u(k)+B*DD;
   x_1=x;
end

if G==1      %Grey prediction
	xx1(1)=x1(2);xx2(1)=x2(2);
	BB=[xx1(1) xx2(1) 1];

for i=2:1:N-2
    xx1(i)=xx1(i-1)+x1(i+1);
    xx2(i)=xx2(i-1)+x2(i+1);
    BB=[BB;xx1(i) xx2(i) i];
end

for i=1:1:N-1
   u(i)=kp*e(i)+kd*de(i);
   DDD(i)=1/B*([x1(i+1);x2(i+1)]-A*[x1(i);x2(i)])-u(i);
end
D1(1)=DDD(2);

for i=2:1:N-2
   D1(i)=D1(i-1)+DDD(i+1);
end
end

xp=[x1' x2'];

if G==1
	ab=abs(det(BB'*BB))
	V1=inv(BB'*BB)*BB'*D1';
	Vp=V1'
end

N=N1;   %If G=2
end
figure(1);grid on;
plot(time,r,'r',time,x1,'b');
xlabel('time(s)');ylabel('r,x1');
figure(2);grid on;
plot(time,r-x1,'b');
xlabel('time(s)');ylabel('error');
figure(3);grid on;
plot(time,u);
xlabel('time(s)');ylabel('u');