%Single Neural Net PID Controller based on RBF Identification
clear all;close all;

Jp=0.0030;bp=0.067;
ts=0.001;

Gp=tf([1],[Jp,bp,0]);
Gpz=c2d(Gp,ts,'z');
[num,den]=tfdata(Gpz,'v');

h=zeros(6,1);
%w=rands(6,1);
w=[-0.5646;
    0.3937;
   -0.5556;
    0.3981;
    0.4495;
    0.2565];
w_1=w;w_2=w;w_3=w;

xite=0.40;
alfa=0.05;
belte=0.01;

x=[0,0,0]';

%c=0.1*ones(3,6);
%b=0.1*ones(6,1);
c=[-3.1829   -0.5211    7.1754   11.6631   -3.6992  -10.9150;
   -3.8909    2.3999    5.1730    8.5871  -11.3737   -7.0179;
   -4.2018    2.6742    5.1828    8.5238   -1.8936   -6.1845];
b=[ 5.3074;
    1.4771;
   26.4114;
   22.1716;
   52.9082;
    5.6906];
c_1=c;c_2=c_1;c_3=c_2;
b_1=b;b_2=b_1;b_3=b_2;

xc=[0,0,0]';
xitec=0.60;

kp=80;
kd=5;
ki=50;

wc=[kp,kd,ki];
wc_1=wc;wc_2=wc;wc_3=wc;

error_1=0;error_2=0;
y_1=0;y_2=0;
u_1=0;u_2=0;

ei=0;
c_size=size(c);

for k=1:1:1000
    time(k)=k*ts;
    rin(k)=0.50*sin(3*2*pi*k*ts);
    
S=2;    
if S==1    
   yrout(k)=1.0*rin(k);
end
if S==2    
	yrout(k)=0.2*y_1+0.6*rin(k); %Reference Model
end

%Linear model
yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
   
for j=1:1:c_size(2),
    h(j)=exp(-norm(x-c_1(:,j))^2/(2*b_1(j)*b_1(j)));
end
    ymout(k)=w_1'*h;
    
id=abs(yout(k)-ymout(k));
if id>0.0001,
%-----------------Adjusting RBF parameters-----------------------%
   d_w=0*w;        % Defining matrix number of d_w equal to that of w
   for j=1:1:6
   d_w(j)=xite*(yout(k)-ymout(k))*h(j);
   end
   w=w_1+d_w+alfa*(w_1-w_2)+belte*(w_2-w_3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   d_b=0*b;
   for j=1:1:6
       d_b(j)=xite*(yout(k)-ymout(k))*w_1(j)*h(j)*(b_1(j)^-3)*norm(x-c_1(:,j))^2;
   end
   b=b_1+ d_b+alfa*(b_1-b_2)+belte*(b_2-b_3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   for j=1:1:6
     for i=1:1:3
         d_c(i,j)=xite*(yout(k)-ymout(k))*w_1(j)*h(j)*(x(i)-c_1(i,j))*(b_1(j)^-2);
     end
   end
   c=c_1+d_c+alfa*(c_1-c_2)+belte*(c_2-c_3);
end 
%%%%%%%%%%% Calculating Jacobian %%%%%%%%%%%%
dyu=0;
for j=1:1:c_size(2)
    dyu=dyu+w(j)*h(j)*(-x(1)+c(1,j))/b(j)^2;
end
dyout(k)=dyu;
%%%%%%Parameters Return%%%%%%%
error(k)=yrout(k)-yout(k);
xc(1)=error(k);
xc(2)=(error(k)-error_1)/ts;
ei=ei+error(k)*ts;
xc(3)=ei;
   
u(k)=wc*xc;  %Control law
if u(k)>10,
   u(k)=10;
end   
if u(k)<-10,
   u(k)=-10;
end   
  
d_wc=0*wc;      % Defining matrix number of d_w equal to that of w
for j=1:1:3
    d_wc(j)=xitec*error(k)*xc(j)*dyout(k);
end
wc=wc_1+d_wc+alfa*(wc_1-wc_2)+belte*(wc_2-wc_3);
   
error_2=error_1;error_1=error(k);
    
u_2=u_1;u_1=u(k);
y_2=y_1;y_1=yout(k);
   
x(3)=y_2;
x(2)=y_1;
x(1)=u_1;
   
w_3=w_2;w_2=w_1;w_1=w;
c_3=c_2;c_2=c_1;c_1=c;
b_3=b_2;b_2=b_1;b_1=b;
wc_3=wc_2;wc_2=wc_1;wc_1=wc;
end
figure(1);
plot(time,yout,'r',time,ymout,'b');
xlabel('time(s)');ylabel('yout,ymout');
figure(2);
plot(time,dyout,'r');
xlabel('time(s)');ylabel('Jacobian value');
figure(3);
plot(time,yrout,'b',time,yout,'r');
xlabel('time(s)');ylabel('yrout,yout');
figure(4);
plot(time,yrout-yout,'r');
xlabel('time(s)');ylabel('control error');