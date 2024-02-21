%PID Control with Repetitive Control compensation
clear all;close all;

ts=0.001;
sys=tf(50,[0.000046,0.006,1,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

Q=tf(1,[0.20,1]); %Filter 
dQ=c2d(Q,ts,'z');
[numQ,denQ]=tfdata(dQ,'v');

F=1;
N=1/F*1/ts;

zz=tf([1],[1 zeros(1,N)],ts);
dz=dQ*zz;
[numz,denz]=tfdata(dz,'v');

Gr=1/(1-dz);

u_1=0;u_2=0;u_3=0;
y_1=0;y_2=0;y_3=0;

ei=0;ei1=0;
ue_1=0;ue_2=0;
ue_N=0;ue_N1=0;ue_N2=0;
e2_N=0;e2_N1=0;e2_N2=0;

e_N1=0;
e1_1=0;
e2_1=0;

for k=1:1:10000
time(k)=k*ts;

rin(k)=1.0*sin(F*2*pi*k*ts);

yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;

e(k)=rin(k)-yout(k);

ei=ei+e(k)*ts;
up(k)=1.5*e(k)+10*ei;

e1(k)=-denQ(2)*e1_1+numQ(2)*e_N1;
ei1=ei1+e1(k)*ts;

e2(k)=2*e1(k)+1.0*ei1;

ue(k)=0.8187*ue_1+0.1813*ue_N1+e2(k)-0.8187*e2_1;

M=2;
if M==1
   u(k)=up(k);        %Only using PID
end
if M==2
   u(k)=ue(k)+up(k);  %Using REP+PID
end

if k>N
   ue_N=ue(k-N);
   e2_N=e2(k-N);
end   
if k>N+1   
   ue_N1=ue(k-N-1);
   e2_N1=e2(k-N-1);
   e_N1=e(k-N-1);
end   
if k>N+2   
   ue_N2=ue(k-N-2);
   e2_N2=e2(k-N-2);
end

e1_1=e1(k);   
e2_1=e2(k);   
ue_2=ue_1;
ue_1=ue(k);

u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
end
figure(1);
plot(time,rin,'k',time,yout,'k');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,rin-yout,'k');ylabel('error');
xlabel('time(s)');ylabel('error');
figure(3);
plot(time,u,'k');
xlabel('time(s)');ylabel('u');
figure(4);
plot(time,up,'k',time,ue,'k');
xlabel('time(s)');ylabel('up,ue');