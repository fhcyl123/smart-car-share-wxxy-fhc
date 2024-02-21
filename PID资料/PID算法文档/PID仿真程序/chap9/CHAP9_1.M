%PID Control based on Lugre friction model
clear all;
close all;

ts=0.001;
u_1=0;
qq=zeros(3,1);

for k=1:1:15000
  
time(k)=k*ts;
rin(k)=0.05*sin(0.1*2*pi*k*ts);
drin(k)=0.05*0.1*2*pi*cos(0.1*2*pi*k*ts);;

%Lugre friction model
[tt,yy]=ode15s('chap9_1eq',[0 ts],qq,[],u_1);
qq=yy(length(yy),:);
yout(k)=qq(1); 
y2(k)=qq(2); 

u(k)=50*(rin(k)-yout(k))+0.010*(drin(k)-y2(k))/ts;
u_1=u(k);
end
figure(1);
plot(time,rin,'r',time,yout,'b');
xlabel('time(s)');ylabel('Position tracking');
figure(2);
plot(time,drin,'r',time,y2,'b');
xlabel('time(s)');ylabel('Speed tracking');