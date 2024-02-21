%Big Delay PID Control with Smith Algorithm
clear all;close all;
Ts=20;

%Delay plant
kp=1;
Tp=60;
tol=80;
sys=tf([kp],[Tp,1],'inputdelay',tol);
dsys=c2d(sys,Ts,'zoh');
[num,den]=tfdata(dsys,'v');

M=2;
%Prediction model
if M==1  %No Precise Model: PI+Smith
   kp1=kp*1.10;
   Tp1=Tp*1.10;
   tol1=tol*1.0;
elseif M==2|M==3  %Precise Model: PI+Smith
   kp1=kp;
   Tp1=Tp;
   tol1=tol;
end

sys1=tf([kp1],[Tp1,1],'inputdelay',tol1);
dsys1=c2d(sys1,Ts,'zoh');
[num1,den1]=tfdata(dsys1,'v');

u_1=0.0;u_2=0.0;u_3=0.0;u_4=0.0;u_5=0.0;
e1_1=0;
e2=0.0;
e2_1=0.0;
ei=0;

xm_1=0.0;
ym_1=0.0;
y_1=0.0;

for k=1:1:600
    time(k)=k*Ts;
   
S=2;
if S==1
   rin(k)=1.0;     %Tracing Step Signal
end
if S==2
   rin(k)=sign(sin(0.0002*2*pi*k*Ts));  %Tracing Square Wave Signal
end

%Prediction model
xm(k)=-den1(2)*xm_1+num1(2)*u_1;
ym(k)=-den1(2)*ym_1+num1(2)*u_5;  %With Delay 

yout(k)=-den(2)*y_1+num(2)*u_5;

if M==1       %No Precise Model: PI+Smith
   e1(k)=rin(k)-yout(k);
   e2(k)=e1(k)-xm(k)+ym(k);
	ei=ei+Ts*e2(k);
	u(k)=0.50*e2(k)+0.010*ei;
   e1_1=e1(k);
elseif M==2   %Precise Model: PI+Smith
   e2(k)=rin(k)-xm(k);
	ei=ei+Ts*e2(k);
	u(k)=0.50*e2(k)+0.010*ei;    
	e2_1=e2(k);
elseif M==3  %Only PI
   e1(k)=rin(k)-yout(k);
	ei=ei+Ts*e1(k);
	u(k)=0.50*e1(k)+0.010*ei;   
   e1_1=e1(k);
end

%----------Return of smith parameters------------
xm_1=xm(k);
ym_1=ym(k);

u_5=u_4;u_4=u_3;u_3=u_2;u_2=u_1;u_1=u(k);
y_1=yout(k);
end
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');