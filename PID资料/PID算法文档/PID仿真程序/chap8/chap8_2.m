%PID Control based on Grey model compensation
clear all;close all;
global a21 a22 b A B kp kd 

para=[];

BB=zeros(1,3);

ts=0.001;
N=3;    %Needing N>=2(2 is x demension number)
TimeSet=[0:ts:ts*N];

%Using grey model to predict disturbance
V=zeros(1,3);
[t,x]=ode45('chap8_2f',TimeSet,[0.5 0.5],[],para,V);
x11=x(:,1);
x22=x(:,2);

   x1=x(2,:);  %It is the first value(not including initial value)
	BB=[x1(1,:),1];
	for k=2:1:N
		 x1=[x1;x1(k-1,:)+x(k+1,:)];
	    BB=[BB;[x1(k,:) k]];
   end

	D=zeros(N+1,1);
   for k=2:1:N+1
      ddx(k)=(x22(k)-x22(k-1))/ts;
      up(k)=b*kp*x11(k)+b*kd*x22(k);
      D(k)=1/b*(ddx(k)-a21*x11(k)-a22*x22(k)-up(k));
	end
	D1=zeros(N,1);
	D1(1)=D(1)+D(2);
	for k=2:1:N
	   D1(k)=D1(k-1)+D(k+1);
   end
   
   V1=inv(BB'*BB)*BB'*D1;
	V=V1'

%Grey PID control using grey prediction results
N1=2000;
TimeSet1=[0:ts:ts*N1];
[t,x]=ode45('chap8_2f',TimeSet1,[0.50 0.50],[],para,V);
x1=x(:,1);
x2=x(:,2);
for k=1:1:N1+1
   kpd=[kp kd];
	up(k)=kpd*[x1(k);x2(k)];
	uc(k)=-V*[x1(k);x2(k);1];   %Grey Compensation
	u(k)=up(k)+uc(k);
end
figure(1);
plot(t,x1);
xlabel('time(s)');ylabel('error');
figure(2);
plot(t,x2);ylabel('derror');
xlabel('time(s)');ylabel('derror');
figure(3);
plot(t,u);
xlabel('time(s)');ylabel('u');