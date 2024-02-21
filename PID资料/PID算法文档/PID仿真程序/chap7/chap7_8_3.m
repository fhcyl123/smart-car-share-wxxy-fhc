%Zero Phase Error Position control
clear all;
close all;
load zpecoeff.mat;  %ZPE coefficient nF and dF
load closed.mat;    %Load kp

ts=0.001;
sys=tf(5.235e005,[1,87.35,1.047e004,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

u_1=0.0;u_2=0.0;u_3=0.0;
rin_5=0;rin_4=0;rin_3=0;rin_2=0;rin_1=0;
rinn_1=0;
y_1=0;y_2=0;y_3=0;
error_1=0;

F=3;
S=1;
for k=1:1:2000
time(k)=k*ts;

if S==1         %Sine Signal
	rin(k)=0.50*sin(F*2*pi*k*ts);
elseif S==2     %Random Signal
	rin(k)=0.50*sin(1*2*pi*k*ts)+1.0*sin(3*2*pi*k*ts)+1.0*sin(5*2*pi*k*ts);
end

rinn(k)=nF(1)*rin(k)+nF(2)*rin_1+nF(3)*rin_2+nF(4)*rin_3+nF(5)*rin_4-dF(2)*rinn_1;

%Linear model
yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k)=rinn(k)-yout(k);
u(k)=kp*error(k);   %P Control

u_3=u_2;u_2=u_1;u_1=u(k);
rin_5=rin_4;rin_4=rin_3;rin_3=rin_2;rin_2=rin_1;rin_1=rin(k);
rinn_1=rinn(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
end
figure(1);
plot(time,rin,'r',time,yout,'b');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,rin-yout,'r');
xlabel('time(s)');ylabel('error');