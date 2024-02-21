%Adaptive PID control based on Fuzzy RBF Identification
clear all;
close all;

xite=0.20;
alfa=0.02;

c0=0.65*ones(2,5);
b0=0.30*ones(5,1);
w0=[0.5632    0.2176    0.5049
    0.5533   -0.5065   -0.7017
   -0.7906    0.8590    0.3218
   -0.3420   -0.1401   -0.5261
   -0.4778   -0.9020    0.4552
   -0.2232    0.8098   -0.4829
    0.7947    0.9565   -0.0610
    0.5336    0.3156    0.7150
   -0.5114    0.4539    0.6668
   -0.0907   -0.0227   -0.0480
   -0.4553    0.3716   -0.3197
   -0.1536   -0.2741    0.0844
    0.3980   -0.1980    0.1230
    0.6349    0.0287    0.8504
    0.6509   -0.9090   -0.4512
   -0.1935    0.9827   -0.4087
    0.3032   -0.3808    0.2251
   -0.6951   -0.4871    0.7813
   -0.3202    0.2202    0.4289
   -0.9614   -0.1060   -0.5970
    0.5366   -0.1631   -0.7837
   -0.0842   -0.1763    0.4761
   -0.9129    0.4281    0.5240
   -0.8009   -0.7944   -0.4420
    0.9138   -0.2397    0.6904];
%w0=rands(25,3);

c=c0;c_1=c0;c_2=c0;
b=b0;b_1=b0;b_2=b0;
w4=w0;w4_1=w0;w4_2=w0;

x=[0,0]';
xc=[0,0,0]';

du_1=0;u_1=0;y_1=0;
error_1=0;error_2=0;error=0;

ts=0.001;
for k=1:1:500
   time(k)=k*ts;
   rin(k)=1.0;
   yout(k)=(-0.1*y_1+u_1)/(1+y_1^2);  %Nonlinear plant
   
   x(1)=rin(k);
   x(2)=yout(k);
   
   f1=x;                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:1:2                         % Layer2:fuzzation
   for j=1:1:5
      net2(i,j)=-(f1(i)-c_1(i,j))^2/b_1(j)^2;
   end
end
for i=1:1:2
   for j=1:1:5
       f2(i,j)=exp(net2(i,j));
   end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:1:5                        % Layer3:fuzzy inference(49 rules)
    m1(j)=f2(1,j);
	 m2(j)=f2(2,j);
end

for i=1:1:5
for j=1:1:5
    ff3(i,j)=m2(i)*m1(j);           
end
end
f3=[ff3(1,:),ff3(2,:),ff3(3,:),ff3(4,:),ff3(5,:)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f4=w4'*f3';                      % Layer4:output

kp(k)=f4(1);
ki(k)=f4(2);
kd(k)=f4(3);

%----------Calculate error id between yout and ymout-----------%
error(k)=rin(k)-yout(k);

du(k)=kp(k)*xc(1)+kd(k)*xc(2)+ki(k)*xc(3); 
u(k)=u_1+du(k);

dyu(k)=sign((yout(k)-y_1)/(du(k)-du_1+0.0001));

d_w4=0*w4_1;
for i=1:1:25
   for j=1:1:3
      d_w4(i,j)=xite*error(k)*dyu(k)*xc(j)*f3(i);
   end
end
w4=w4_1+ d_w4+alfa*(w4_1-w4_2);

%Return of parameters
	du_1=du(k);	
   u_1=u(k);
   y_1=yout(k);
   
   w4_2=w4_1;w4_1=w4;
   
   xc(1)=error(k)-error_1;             %Calculating P
   xc(2)=error(k);                     %Calculating I
   xc(3)=error(k)-2*error_1+error_2;   %Calculating D
   
   error_2=error_1;
   error_1=error(k);
end
figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
subplot(311);
plot(time,kp,'r');
xlabel('time(s)');ylabel('kp');
subplot(312);
plot(time,ki,'r');
xlabel('time(s)');ylabel('ki');
subplot(313);
plot(time,kd,'r');
xlabel('time(s)');ylabel('kd');