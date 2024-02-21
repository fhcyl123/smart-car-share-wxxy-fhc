%Fuzzy Controller
clear all;close all;

a=newfis('fuzz_ljk');

f1=1.0;
a=addvar(a,'input','e',[-3*f1,3*f1]);                     % Parameter e
a=addmf(a,'input',1,'NB','zmf',[-3*f1,-1*f1]);
a=addmf(a,'input',1,'NM','trimf',[-3*f1,-2*f1,0]);
a=addmf(a,'input',1,'NS','trimf',[-3*f1,-1*f1,1*f1]);
a=addmf(a,'input',1,'Z','trimf',[-2*f1,0,2*f1]);
a=addmf(a,'input',1,'PS','trimf',[-1*f1,1*f1,3*f1]);
a=addmf(a,'input',1,'PM','trimf',[0,2*f1,3*f1]);
a=addmf(a,'input',1,'PB','smf',[1*f1,3*f1]);

f2=1.0;
a=addvar(a,'input','ec',[-3*f2,3*f2]);                     % Parameter ec
a=addmf(a,'input',2,'NB','zmf',[-3*f2,-1*f2]);
a=addmf(a,'input',2,'NM','trimf',[-3*f2,-2*f2,0]);
a=addmf(a,'input',2,'NS','trimf',[-3*f2,-1*f2,1*f2]);
a=addmf(a,'input',2,'Z','trimf',[-2*f2,0,2*f2]);
a=addmf(a,'input',2,'PS','trimf',[-1*f2,1*f2,3*f2]);
a=addmf(a,'input',2,'PM','trimf',[0,2*f2,3*f2]);
a=addmf(a,'input',2,'PB','smf',[1*f2,3*f2]);

%f3=1.4;
f3=1.5;
a=addvar(a,'output','u',[-3*f3,3*f3]);                     % Parameter u
a=addmf(a,'output',1,'NB','zmf',[-3*f3,-1*f3]);
a=addmf(a,'output',1,'NM','trimf',[-3*f3,-2*f3,0]);
a=addmf(a,'output',1,'NS','trimf',[-3*f3,-1*f3,1*f3]);
a=addmf(a,'output',1,'Z','trimf',[-2*f3,0,2*f3]);
a=addmf(a,'output',1,'PS','trimf',[-1*f3,1*f3,3*f3]);
a=addmf(a,'output',1,'PM','trimf',[0,2*f3,3*f3]);
a=addmf(a,'output',1,'PB','smf',[1*f3,3*f3]);

%Each rule is a PD rule: error=rin-yout (nagative feedback)

rulelist=[1 1 7 1 1;                               % Edit rule base
          1 2 7 1 1;
          1 3 6 1 1;
          1 4 6 1 1;
          1 5 5 1 1;
          1 6 5 1 1;
          1 7 4 1 1;
   
          2 1 7 1 1;
          2 2 6 1 1;
          2 3 6 1 1;
          2 4 5 1 1;
          2 5 5 1 1;
          2 6 4 1 1;
          2 7 3 1 1;
          
          3 1 6 1 1;
          3 2 6 1 1;
          3 3 5 1 1;
          3 4 5 1 1;
          3 5 4 1 1;
          3 6 3 1 1;
          3 7 3 1 1;
          
          4 1 6 1 1;
          4 2 5 1 1;
          4 3 5 1 1;
          4 4 4 1 1;
          4 5 3 1 1;
          4 6 3 1 1;
          4 7 2 1 1;
          
          5 1 5 1 1;
          5 2 5 1 1;
          5 3 4 1 1;
          5 4 3 1 1;
          5 5 3 1 1;
          5 6 2 1 1;
          5 7 2 1 1;
          
          6 1 5 1 1;
          6 2 5 1 1;
          6 3 4 1 1;
          6 4 3 1 1;
          6 5 2 1 1;
          6 6 2 1 1;
          6 7 1 1 1;
       
          7 1 4 1 1;
          7 2 3 1 1;
          7 3 3 1 1;
          7 4 2 1 1;
          7 5 2 1 1;
          7 6 1 1 1;
          7 7 1 1 1];
          
a=addrule(a,rulelist);
%showrule(a)                        % Show fuzzy rule base

a1=setfis(a,'DefuzzMethod','centroid');  % Defuzzy
writefis(a1,'ljk');                 % save to fuzzy file "ljk.fis" which can be
                                    % simulated with fuzzy tool
a2=readfis('ljk');
%plotfis(a2);

%fuzzy ljk.fis    %Demo fuzzy control simulation
%ruleview(a2);

%%%%%%%%%%%%%%%%%% Using Fuzzy Controller%%%%%%%%%%%%%%%%%
sys=tf(5.235e005,[1,87.35,1.047e004,0]);
dsys=c2d(sys,0.001,'z');
[num,den]=tfdata(dsys,'v');
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

error_1=0;
e_1=0.0;
ec_1=0.0;
ei=0;
ts=0.001;
%----------------------------------------
%  Start of Control
%----------------------------------------
for k=1:1:2000
time(k)=k*ts;

rin(k)=1*sign(sin(1*2*pi*k*ts));       %Tracing Fangbo Signal

yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k)=yout(k)-rin(k);
ei=ei+error(k)*ts;

u(k)=evalfis([e_1 ec_1],a2);     %Using fuzzy inference
   
u_3=u_2;
u_2=u_1;
u_1=u(k);
   
y_3=y_2;
y_2=y_1;
y_1=yout(k);
   
e_1=error(k);
%ec_1=(error(k)-error_1)/ts;
ec_1=error(k)-error_1;
   
error_2=error_1;
error_1=error(k);
end
figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('Time(second)');ylabel('rin,yout');
figure(2);
plot(time,u,'r');
xlabel('Time(second)');ylabel('u');