%Grey model prediction
clear all;close all;
global J b

para=[];

BB=zeros(1,3);

ts=0.001;
N=3;    %Needing N>=2(2 is x demension number)
TimeSet=[0:ts:ts*N];

%Using grey model to predict disturbance
V=zeros(1,3);
para=[];
[t,x]=ode45('chap8_1eq',TimeSet,[0 0],[],para,V);
x22=x(:,2);    %Speed value

   x1(1,:)=x(2,:);  %It is the first value (not including initial value)
	BB=[x1(1,:),1];
	for k=2:1:N
       x1=[x1;
           x1(k-1,:)+x(k+1,:)];
       BB=[BB;
          [x1(k,:) k]];
	end

	D=zeros(N+1,1);
   for k=2:1:N+1
      ddx(k)=(x22(k)-x22(k-1))/ts;
      u(k)=0.50*sin(k*ts);
      D(k)=1/b*(ddx(k)+J*x22(k))-u(k);
	end
   
   D1=zeros(N,1);
	D1(1)=D(1)+D(2);
	for k=2:1:N
	    D1(k)=D1(k-1)+D(k+1);
   end
   
   V=inv(BB'*BB)*BB'*D1;
   V=V'