%CMAC and PID Concurrent Control
clear all;
close all;

ts=0.001;
sys=tf(1770,[1,60,1770]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

alfa=0.04;

N=100;C=5;

w=zeros(N+C,1);
w_1=w;w_2=w;d_w=w;

y_1=0;y_2=0;y_3=0;
u_1=0.0;u_2=0.0;u_3=0.0;

x=[0,0,0]';
error_1=0;

%Square Wave Signal
	A=0.50;     
	Smin=-A;
	Smax=A;
	xite=0.10; 
	kp=25;
	ki=0.0;
	kd=0.28;

%Coding Input Value
dvi=(Smax-Smin)/(N-1);

for i=1:1:C                %C size
    v(i)=Smin;
end
for i=C+1:1:C+N            %N size
    v(i)=v(i-1)+dvi;
end
for i=N+C+1:1:N+2*C        %C size
    v(i)=Smax;
end

for k=1:1:1000
    time(k)=k*ts;
    
rin(k)=A*sign(sin(2*2*pi*k*ts));      %Square Signal

for i=1:1:N+C
if rin(k)>=v(i)&rin(k)<=v(i+C)
   a(i)=1;
else
   a(i)=0;
end
end

yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;

error(k)=rin(k)-yout(k);

%CMAC Neural Network Controller
un(k)=a*w;

%PID Controller
up(k)=kp*x(1)+kd*x(2)+ki*x(3);

M=2;
if M==1      %Only Using PID Control
   u(k)=up(k); 
elseif M==2  %Total control output
	u(k)=up(k)+un(k);
end

if k==150     %Disturbance
   u(k)=u(k)+5.0;
end

if u(k)>=10
   u(k)=10;
end
if u(k)<=-10
   u(k)=-10;
end   

%Update NN Weight
d_w=a'*xite*(u(k)-un(k))/C;

w=w_1+ d_w+alfa*(w_1-w_2);

%Parameters Update
w_3=w_2;w_2=w_1;w_1=w;
u_2=u_1;u_1=u(k);
y_2=y_1;y_1=yout(k);

x(1)=error(k);                % Calculating P
x(2)=(error(k)-error_1)/ts;   % Calculating D
x(3)=x(3)+error(k)*ts;        % Calculating I
   
error_2=error_1;error_1=error(k);
end
figure(1);
plot(time,rin,'k',time,yout,'k');
xlabel('time(s)');ylabel('(rin and y)');
figure(2);
subplot(311);
plot(time,un,'k');
xlabel('time(s)');ylabel('un');
subplot(312);
plot(time,up,'k');
xlabel('time(s)');ylabel('up');
subplot(313);
plot(time,u,'k');
xlabel('time(s)');ylabel('u');
figure(3);
plot(time,error,'k');
xlabel('time(s)');ylabel('error');