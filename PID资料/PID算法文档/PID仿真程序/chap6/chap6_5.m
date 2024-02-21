%DRNN Tunning PID Controler for coupling plant
clear all;
close all;

u1_1=0.0;u1_2=0.0;u1_3=0.0;
u2_1=0.0;u2_2=0.0;u2_3=0.0;
y1_1=0;y2_1=0;

wd1=rands(7,1);
wo1=rands(7,1);
wi1=rands(3,7);
wd1_1=wd1;wo1_1=wo1;wi1_1=wi1; 

x1=zeros(7,1);
x1_1=x1;

wd2=rands(7,1);
wo2=rands(7,1);
wi2=rands(3,7);
wd2_1=wd2;wo2_1=wo2;wi2_1=wi2; 

xitei1=0.40;xited1=0.40;xiteo1=0.40;
xitei2=0.40;xited2=0.40;xiteo2=0.40;
alfa1=0.04;alfa2=0.04;

x2=zeros(7,1);
x2_1=x2;

error1_1=0;error1_2=0;
error2_1=0;error2_2=0;

kp1=0.30;ki1=0.150;kd1=0.2;
kp2=0.30;ki2=0.150;kd2=0.2;

kp1_1=kp1;kd1_1=kd1;ki1_1=ki1;  
kp2_1=kp2;kd2_1=kd2;ki2_1=ki2;  

xitekp1=0.5;xitekd1=0.3;xiteki1=0.001;
xitekp2=0.5;xitekd2=0.3;xiteki2=0.0001;

ei1=0;ei2=0;
ts=1;
for k=1:1:1500
time(k)=k*ts;

%Step Signal
R=[1;0];
%R=[0;1];

%Coupling Plant
yout1(k)=1.0/(1+y1_1)^2*(0.8*y1_1+u1_2+0.2*u2_3);
yout2(k)=1.0/(1+y2_1)^2*(0.9*y2_1+0.3*u1_3+u2_2);

In1=[u1_1,yout1(k),1]';
for j=1:1:7
    si(j)=In1'*wi1(:,j)+wd1(j)*x1(j);
end

for j=1:1:7
    x1(j)=(1-exp(-si(j)))/(1+exp(-si(j)));
end

Pi=0*x1;
for j=1:1:7
    Pi(j)=wo1(j)*(1+x1(j))*(1-x1(j))*x1_1(j);
end

Qi=0*wi1;
for j=1:1:7
   for i=1:1:3
       Qi(i,j)=wo1(j)*(1+x1(j))*(1-x1(j))*In1(i);
   end
end   

ym=0;
for j=1:1:7
    ym=ym+x1(j)*wo1(j);
end
ymout1(k)=ym;

wo1=wo1+xiteo1*(yout1(k)-ymout1(k))*x1+alfa1*(wo1-wo1_1);
wd1=wd1+xited1*(yout1(k)-ymout1(k))*Pi+alfa1*(wd1-wd1_1); 
for j=1:1:7
   if abs(wd1(j))>1
      wd1(j)=0.5*sign(wd1(j));
   end
end   
wi1=wi1+xitei1*(yout1(k)-ymout1(k))*Qi+alfa1*(wi1-wi1_1);   
  
yu=0;
for j=1:1:7
   yu=yu+wo1(j)*wi1(1,j)*(1+x1(j))*(1-x1(j));
end   
dyout1(k)=yu;

error1(k)=R(1)-yout1(k);
  
xc1(1)=error1(k);             %Calculating P
xc1(2)=error1(k)-error1_1;    %Calculating D
ei1=ei1+error1(k)*ts;
xc1(3)=ei1;                   %Calculating I
      
kp1(k)=kp1_1+xitekp1*error1(k)*xc1(1);
kd1(k)=kd1_1+xitekd1*error1(k)*xc1(2);
ki1(k)=ki1_1+xiteki1*error1(k)*xc1(3);  

kp1(k)=kp1_1+xitekp1*error1(k)*xc1(1)*dyout1(k);
kd1(k)=kd1_1+xitekd1*error1(k)*xc1(2)*dyout1(k);
ki1(k)=ki1_1+xiteki1*error1(k)*xc1(3)*dyout1(k);  

if kp1(k)<0
   kp1(k)=0;
end
if kd1(k)<0
   kd1(k)=0;
end
if ki1(k)<0
   ki1(k)=0;
end
   
u1(k)=kp1(k)*xc1(1)+kd1(k)*xc1(2)+ki1(k)*xc1(3); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
In2=[u2_1,yout2(k),1]';

for j=1:1:7
   si(j)=In1'*wi2(:,j)+wd2(j)*x2(j);
end

for j=1:1:7
   x2(j)=(1-exp(-si(j)))/(1+exp(-si(j)));
end

Pi=0*x2;
for j=1:1:7
   Pi(j)=wo2(j)*(1+x2(j))*(1-x2(j))*x2_1(j);
end

Qi=0*wi2;
for j=1:1:7
   for i=1:1:3
      Qi(i,j)=wo2(j)*(1+x2(j))*(1-x2(j))*In2(i);
   end
end   

ym=0;
for j=1:1:7
   ym=ym+x2(j)*wo2(j);
end
ymout2(k)=ym;

wo2=wo2+xiteo2*(yout2(k)-ymout2(k))*x2+alfa2*(wo2-wo2_1);
wd2=wd2+xited2*(yout2(k)-ymout2(k))*Pi+alfa2*(wd2-wd2_1);
  for j=1:1:7
     if abs(wd2(j))>1
        wd2(j)=0.5*sign(wd2(j));
     end
  end   
wi2=wi2+xitei2*(yout2(k)-ymout2(k))*Qi+alfa2*(wi2-wi2_1);   
  
yu=0;
for j=1:1:7
    yu=yu+wo2(j)*wi2(1,j)*(1+x2(j))*(1-x2(j));
end   
dyout2(k)=yu;

error2(k)=R(2)-yout2(k);

xc2(1)=error2(k);             %Calculating P
xc2(2)=error2(k)-error2_1;    %Calculating D
ei2=ei2+error2(k)*ts;
xc2(3)=ei2;                   %Calculating I
      
kp2(k)=kp2_1+xitekp2*error2(k)*xc2(1)*dyout2(k);
kd2(k)=kd2_1+xitekd2*error2(k)*xc2(2)*dyout2(k);
ki2(k)=ki2_1+xiteki2*error2(k)*xc2(3)*dyout2(k);  

if kp2(k)<0
   kp2(k)=0;
end
if kd2(k)<0
   kd2(k)=0;
end
if ki2(k)<0
   ki2(k)=0;
end
   
u2(k)=kp2(k)*xc2(1)+kd2(k)*xc2(2)+ki2(k)*xc2(3);

%Return of PID parameters
error1_2=error1_1;
error1_1=error1(k);

error2_2=error2_1;
error2_1=error2(k);

wd1_1=wd1;wo1_1=wo1;wi1_1=wi1; 
wd2_1=wd2;wo2_1=wo2;wi2_1=wi2; 

u1_4=u1_3;u1_3=u1_2;u1_2=u1_1;u1_1=u1(k);
u2_4=u2_3;u2_3=u2_2;u2_2=u2_1;u2_1=u2(k);

y1_1=yout1(k);
y2_1=yout2(k);
end
figure(1);
plot(time,dyout1,'r');
xlabel('time(s)');ylabel('Jacobian');
hold on;
plot(time,dyout2,'b');
xlabel('time(s)');ylabel('Jacobian');
figure(2);
plot(time,R(1),'b',time,yout1,'r');
hold on;
plot(time,R(2),'r',time,yout2,'b');
xlabel('time(s)');ylabel('rin,yout');
figure(3);
plot(time,kp1,'r',time,ki1,'b',time,kd1,'k');
xlabel('time(s)');ylabel('kp1 ki1 kd1');
figure(4);
plot(time,kp2,'r',time,ki2,'b',time,kd2,'k');
xlabel('time(s)');ylabel('kp2 ki2 kd2');
