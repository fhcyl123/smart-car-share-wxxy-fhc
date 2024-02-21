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

for l1=1:1:3
   gs1=-[(e(k)+pi/6-(l1-1)*pi/6)/(pi/12)]^2;
	u1(l1)=exp(gs1);
end

for l2=1:1:3
   gs2=-[(de(k)+pi/6-(l2-1)*pi/6)/(pi/12)]^2;
	u2(l2)=exp(gs2);
end
 
U=[-20 -10  0
   -10  0  10
    0  10  20];

fnum=0;
fden=0;
for i=1:1:3
	for j=1:1:3
		fnum=fnum+u1(i)*u2(j)*U(i,j);
		fden=fden+u1(i)*u2(j);
	end
end

u(k)=fnum/(fden+0.01);

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