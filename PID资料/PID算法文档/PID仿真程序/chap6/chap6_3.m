%Single Neural Net PID Decouple Controller based on Hebb Learning 
%Algorithm to adjust kp,ki,kd
clear all;
close all;

xc1=[0,0,0]';
xc2=[0,0,0]';

xiteP=0.40;
xiteI=0.40;
xiteD=0.40;

%Initilizing kp,ki and kd
%Radom Value
%wkp1_1=rand;wki1_1=rand;wkd1_1=rand;
%wkp2_1=rand;wki2_1=rand;wkd2_1=rand;

wkp1_1=0.3150;wki1_1=1.1615;wkd1_1=1.4948;
wkp2_1=0.2067;wki2_1=0.6365;wkd2_1=0.4996;

error1_1=0;error1_2=0;
error2_1=0;error2_2=0;

u1_1=0.0;u1_2=0.0;u1_3=0.0;u1_4=0.0;
u2_1=0.0;u2_2=0.0;u2_3=0.0;u2_4=0.0;

y1_1=0;y2_1=0;

ts=1;
for k=1:1:1500
time(k)=k*ts;

%Step Signal
%R=[1;0];
R=[0;1];

%------------ Calculating practical output --------------%
%Coupling Plant
yout1(k)=1.0/(1+y1_1)^2*(0.8*y1_1+u1_2+0.2*u2_3);
yout2(k)=1.0/(1+y2_1)^2*(0.9*y2_1+0.3*u1_3+u2_2);

error1(k)=R(1)-yout1(k);
error2(k)=R(2)-yout2(k);

%For Variable1
%Adjusting NNC Weight Value by adopting hebb learning algorithm
   wkp1(k)=wkp1_1+xiteP*error1(k)*u1_1*xc1(1);  %P
   wki1(k)=wki1_1+xiteI*error1(k)*u1_1*xc1(2);  %I 
   wkd1(k)=wkd1_1+xiteD*error1(k)*u1_1*xc1(3);  %D
   xc1(1)=error1(k)-error1_1;                   %P
   xc1(2)=error1(k);                            %I
   xc1(3)=(error1(k)-2*error1_1+error1_2);      %D

   wadd1(k)=abs(wkp1(k))+abs(wki1(k))+abs(wkd1(k));
   w111(k)=wkp1(k)/wadd1(k);
   w122(k)=wki1(k)/wadd1(k);
   w133(k)=wkd1(k)/wadd1(k);
   w1=[w111(k),w122(k),w133(k)];
   k1=0.16;
   u1(k)=u1_1+k1*w1*xc1;
   
%For Variable2
%Adjusting NNC Weight Value by adopting hebb learning algorithm
   wkp2(k)=wkp2_1+xiteP*error2(k)*u2_1*xc2(1);  %P
   wki2(k)=wki2_1+xiteI*error2(k)*u2_1*xc2(2);  %I 
   wkd2(k)=wkd2_1+xiteD*error2(k)*u2_1*xc2(3);  %D
   xc2(1)=error2(k)-error2_1;                   %P
   xc2(2)=error2(k);                            %I
   xc2(3)=(error2(k)-2*error2_1+error2_2);      %D

   wadd2(k)=abs(wkp2(k))+abs(wki2(k))+abs(wkd2(k));
   w211(k)=wkp2(k)/wadd2(k);
   w222(k)=wki2(k)/wadd2(k);
   w233(k)=wkd2(k)/wadd2(k);
   w2=[w211(k),w222(k),w233(k)];
   k2=0.16;
   u2(k)=u2_1+k2*w1*xc2;

%-----------Return of PID parameters------------%
%For Variable1
error1_2=error1_1;
error1_1=error1(k);
wkp1_1=wkp1(k);
wkd1_1=wkd1(k);
wki1_1=wki1(k);

u1_4=u1_3;
u1_3=u1_2;
u1_2=u1_1;
u1_1=u1(k);

y1_1=yout1(k);

%For Variable2
error2_2=error2_1;
error2_1=error2(k);
wkp2_1=wkp2(k);
wkd2_1=wkd2(k);
wki2_1=wki2(k);

u2_4=u2_3;
u2_3=u2_2;
u2_2=u2_1;
u2_1=u2(k);

y2_1=yout2(k);
end
figure(1);
plot(time,R(1),'k',time,yout1,'k');
hold on;
plot(time,R(2),'k',time,yout2,'k');
xlabel('time(s)');ylabel('rin,yout');