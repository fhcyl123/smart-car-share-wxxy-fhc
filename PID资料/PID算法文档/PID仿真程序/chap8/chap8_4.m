%Grey model PID Control
clear all;close all;
global kp kd a b A B F AA

F=1.0;
para=[];
AA=0.50;

%Disturbance Prediction
BB=zeros(1,3);
ts=0.001;
N=4;
ab=abs(det(BB'*BB));
   TimeSet=[0:ts:ts*N];
   V=zeros(1,3);
	[t,y]=ode45('chap8_4f',TimeSet,[0 0],[],para,V);  %Grey Prediction
   
   w=2*pi*F;
   r=AA*(sin(w*t));
   dr=AA*w*cos(w*t);
   
   y0=y;
   y1=y0(2,:);
   BB=[y1(1,:),1];
   
	for k=2:1:N
      y1=[y1;y1(k-1,:)+y0(k+1,:)];
      BB=[BB;[y1(k,:) k]];
   end

y11=y(:,1);
y22=y(:,2);

   D=zeros(N+1,1);
   for k=2:1:N+1
      D(k)=1/b*((y22(k)-y22(k-1))/ts-a*y22(k)-b*kp*(r(k)-y11(k))-b*kd*(dr(k)-y22(k)));
	end
   
   D1=zeros(N,1);
   D1(1)=D(2);
     
   for k=2:1:N
      D1(k)=D1(k-1)+D(k+1);
   end
   ab=abs(det(BB'*BB))
     
V1=inv(BB'*BB)*BB'*D1;
V=V1'

%PID Control
N1=2000;
TimeSet1=[0:ts:ts*N1];
[t,y]=ode45('chap8_4f',TimeSet1,[0 0],[],para,V);

y1=y(:,1);
y2=y(:,2);
r=AA*(sin(w*t));
dr=AA*w*cos(w*t);
ddr=-AA*w^2*sin(w*t);

e=r-y1;
de=dr-y2;
for k=1:1:N1+1
	up(k)=kp*e(k)+kd*de(k);
	uc(k)=-V*[y1(k);y2(k);1];   %Grey Compensation
	u(k)=up(k)+uc(k);
end
figure(1);grid on;
plot(t,r,'r',t,y(:,1),'b');
xlabel('time(s)');ylabel('r,y');

figure(2);grid on;
plot(t,r-y(:,1));
xlabel('time(s)');ylabel('error');

figure(3);grid on;
plot(t,u);
xlabel('time(s)');ylabel('u');