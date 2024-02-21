%Dynamic model
function dx=DynamicModel(t,x,flag,para,V)
global a21 a22 b A B kp kd 
dx=zeros(2,1);

a21=0;a22=-25;
b=133;

B=[0;b];
A=[0 1;a21 a22];

V1=[5 5];   
f=5;

DD=V1*x+f;     %System true disturbance
%Control law
kp=-35;
kd=-5;
up=kp*x(1)+kd*x(2);   

M=1;
if M==1
   uc=0;          %No Grey Compensation
elseif M==2
   uc=-V*[x;1];   %Grey Compensation
end
u=up+uc;
dx=A*x+B*(u+DD);