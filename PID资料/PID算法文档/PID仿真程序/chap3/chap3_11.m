%PD Type Fuzzy Controller Design
clear all;
close all;

ts=0.001;

sys=tf(133,[1,25,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

e_1=0;
u_1=0;u_2=0;
y_1=0;y_2=0;

for k=1:1:10000
time(k)=k*ts;

rin(k)=0.5*sin(1*pi*k*ts);

yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
   
e(k)=rin(k)-yout(k);
de(k)=(e(k)-e_1)/ts;

u(k)=chap3_11f(e(k),de(k));

e_1=e(k);
u_2=u_1;u_1=u(k);
y_2=y_1;y_1=yout(k);
end
figure(1);
plot(time,rin,'r',time,yout,'b');
xlabel('Time(s)');ylabel('r,yout');

figure(2);
plot(time,e,'r');
xlabel('Time(s)');ylabel('e');

figure(3);
plot(time,de,'r');
xlabel('Time(s)');ylabel('de');

figure(4);
plot(time,u,'r');
xlabel('Time(s)');ylabel('u');