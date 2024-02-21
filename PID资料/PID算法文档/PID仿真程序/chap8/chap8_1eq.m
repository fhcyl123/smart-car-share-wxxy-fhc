function dx=DynamicModel(t,x,flag,para,V)
global J b
dx=zeros(2,1);

J=25;b=133;

M=4;
if M==1
   V=[0.1 0.1];f=0.1;
elseif M==2
   V=[0.3 0.3];f=0.5;
elseif M==3
   V=[0.5 0.5];f=1.5;
elseif M==4
   V=[1.5 1.5];f=5;
end

DD=V(1)*x(1)+V(2)*x(2)+f;     %System true disturbance

u=0.50*sin(t);      %Give a sine signal force
dx(1)=x(2);
dx(2)=-J*x(2)+b*(u+DD);