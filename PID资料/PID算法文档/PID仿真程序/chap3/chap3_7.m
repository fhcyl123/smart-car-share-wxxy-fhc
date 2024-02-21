%Fuzzy Immune PID Control
clear all;
close all;

a=newfis('fuzz_ljk');

f1=10;
a=addvar(a,'input','u',[-f1*1,f1*1]);             %Parameter e
a=addmf(a,'input',1,'NB','zmf',[-f1*1,f1*1]);
a=addmf(a,'input',1,'PB','smf',[-f1*1,f1*1]);

f2=1.0;
a=addvar(a,'input','du',[-f2*1,f2*1]);            %Parameter ec
a=addmf(a,'input',2,'NB','zmf',[-f2*1,f2*1]);
a=addmf(a,'input',2,'PB','smf',[-f2*1,f2*1]);

f3=1.0;
a=addvar(a,'output','f',[-f3*1,f3*1]);            %Parameter u
a=addmf(a,'output',1,'NB','zmf',[-f3*1,0]);
a=addmf(a,'output',1,'Z','trimf',[-f3*1,0,f3*1]);
a=addmf(a,'output',1,'PB','smf',[0,f3*1]);

rulelist=[2 2 1 1 1;     % Edit rule base
          2 1 2 1 1;
          1 2 2 1 1;
          1 1 3 1 1;];
          
a=addrule(a,rulelist);
%showrule(a)             % Show fuzzy rule base

a1=setfis(a,'DefuzzMethod','centroid');  % Defuzzy
writefis(a1,'ljk');      % save to fuzzy file "ljk.fis" which can be
                         % simulated with fuzzy tool
a2=readfis('ljk');
%plotfis(a2);

%%%%%%%%%%%%%%%%%% Using Fuzzy Controller%%%%%%%%%%%%%%%%%
ts=20;
sys=tf([1],[60,1],'inputdelay',80);
dsys=c2d(sys,ts,'zoh');
[num,den]=tfdata(dsys,'v');

u_1=0;u_2=0;u_3=0;u_4=0;u_5=0;
y_1=0;
e_1=0;e_2=0;
for k=1:1:1000
time(k)=k*ts;

rin(k)=1.0*sign(sin(3*pi*k*0.001));

%Linear model
yout(k)=-den(2)*y_1+num(2)*u_5;

e(k)=rin(k)-yout(k);
ec(k)=e(k)-e_1;

f(k)=evalfis([u_1 u_1-u_2],a2);

K=0.30;
xite=0.80;
Kp(k)=K*(1-xite*f(k));

u(k)=u_1+Kp(k)*((e(k)-e_1)+0.3*e(k)+0.3*(e(k)-2*e_1+e_2));

if k==500
   u(k)=u(k)+1.0;
end

%Return of parameters
u_5=u_4;u_4=u_3;u_3=u_2;u_2=u_1;u_1=u(k);
y_1=yout(k);
e_2=e_1;
e_1=e(k);
end
figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,e,'r');
xlabel('time(s)');ylabel('error');
figure(3);
plot(time,u,'r');
xlabel('time(s)');ylabel('u');
figure(4);
plot(time,Kp,'r');
xlabel('time(s)');ylabel('Kp');
figure(5);
plotmf(a,'input',1);
figure(6);
plotmf(a,'input',2);
figure(7);
plotmf(a,'output',1);
