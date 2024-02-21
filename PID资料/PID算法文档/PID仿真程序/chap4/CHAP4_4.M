%Adaptive PID control based on RBF Identification
clear all;
close all;

xite=0.25;
alfa=0.05;
belte=0.01;
x=[0,0,0]';

ci=30*ones(3,6);
bi=40*ones(6,1);
w=10*ones(6,1); 

h=[0,0,0,0,0,0]';
    
ci_1=ci;ci_3=ci_1;ci_2=ci_1;
bi_1=bi;bi_2=bi_1;bi_3=bi_2;
w_1=w;w_2=w_1;w_3=w_1;

u_1=0;y_1=0;
xc=[0,0,0]';
error_1=0;error_2=0;error=0;

%kp=rand(1);        
%ki=rand(1);     
%kd=rand(1);
kp0=0.03;        
ki0=0.01;     
kd0=0.03;

kp_1=kp0;
kd_1=kd0;
ki_1=ki0;  

xitekp=0.20;
xitekd=0.20;
xiteki=0.20;

ts=0.001;
for k=1:1:2000
   time(k)=k*ts;
   rin(k)=1.0*sign(sin(2*pi*k*ts));
   yout(k)=(-0.1*y_1+u_1)/(1+y_1^2);  %Nonlinear plant
   
   for j=1:1:6
      h(j)=exp(-norm(x-ci(:,j))^2/(2*bi(j)*bi(j)));
   end
   ymout(k)=w'*h;         

   d_w=0*w;
   for j=1:1:6
      d_w(j)=xite*(yout(k)-ymout(k))*h(j);
   end
   w=w_1+d_w+alfa*(w_1-w_2)+belte*(w_2-w_3);
   
   d_bi=0*bi;
   for j=1:1:6
      d_bi(j)=xite*(yout(k)-ymout(k))*w(j)*h(j)*(bi(j)^-3)*norm(x-ci(:,j))^2;
   end
   bi=bi_1+ d_bi+alfa*(bi_1-bi_2)+belte*(bi_2-bi_3);
   for j=1:1:6
     for i=1:1:3
      d_ci(i,j)=xite*(yout(k)-ymout(k))*w(j)*h(j)*(x(i)-ci(i,j))*(bi(j)^-2);
     end
   end
   ci=ci_1+d_ci+alfa*(ci_1-ci_2)+belte*(ci_2-ci_3);
  
%%%%%%%%%%%%%%%%%%%%%%Jacobian%%%%%%%%%%%%%%%%%%%%%%%
  yu=0;
  for  j=1:1:6
      yu=yu+w(j)*h(j)*(-x(1)+ci(1,j))/bi(j)^2; 
  end
  dyout(k)=yu;
%%%%%%%%%%%%%%%%%%%%%%Start of Control system%%%%%%%%%%%%%%%%%%
   error(k)=rin(k)-yout(k);
   kp(k)=kp_1+xitekp*error(k)*dyout(k)*xc(1);
   kd(k)=kd_1+xitekd*error(k)*dyout(k)*xc(2);
   ki(k)=ki_1+xiteki*error(k)*dyout(k)*xc(3);  
   if kp(k)<0
      kp(k)=0;
   end
   if kd(k)<0
      kd(k)=0;
   end
   if ki(k)<0
      ki(k)=0;
   end
   
   M=0;
   switch M
   case 0
   case 1  %Only PID Control
		kp(k)=kp0;
		ki(k)=ki0;     
		kd(k)=kd0;
   end
	du(k)=kp(k)*xc(1)+kd(k)*xc(2)+ki(k)*xc(3); 
   u(k)=u_1+du(k);

%Return of parameters
   x(1)=du(k);
   x(2)=yout(k);
   x(3)=y_1;

   
	u_1=u(k);
   y_1=yout(k);
   
   ci_3=ci_2;
   ci_2=ci_1;
   ci_1=ci;
   
   bi_3=bi_2;
   bi_2=bi_1;
   bi_1=bi;
   
   w_3=w_2;
   w_2=w_1;
   w_1=w;
   
   xc(1)=error(k)-error_1;             %Calculating P
   xc(2)=error(k)-2*error_1+error_2;   %Calculating D
   xc(3)=error(k);                     %Calculating I
   
   error_2=error_1;
   error_1=error(k);
   
   kp_1=kp(k);
   kd_1=kd(k);
   ki_1=ki(k);  
end
figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,yout,'r',time,ymout,'b');
xlabel('time(s)');ylabel('yout,ymout');
figure(3);
plot(time,dyout);
xlabel('time(s)');ylabel('Jacobian value');
figure(4);
subplot(311);
plot(time,kp,'r');
xlabel('time(s)');ylabel('kp');
subplot(312);
plot(time,ki,'r');
xlabel('time(s)');ylabel('ki');
subplot(313);
plot(time,kd,'r');
xlabel('time(s)');ylabel('kd');