%Dynamic model
function dy=DynamicModel(t,y,flag,para,V)
global kp kd a b A B F AA
dy=zeros(2,1);

%Input signal
w=2*pi*F;
rr=AA*sin(w*t);
dr=AA*w*cos(w*t);
ddr=-AA*(w^2)*sin(w*t);

r=[rr;dr];

b=133;a=-25;
B=[0;b];
A=[0 1;0 a];

V1=[5 5];f=5;

DD=V1*y+f;      %True disturbance
%Control law
kp=30;kd=5.0;kpd=[kp,kd];
up=kpd*(r-y);

M=2;
if M==1
   uc=0;          %No Grey Compensation
elseif M==2
   uc=-V*[y;1];   %Grey Compensation
end
u=up+uc;
dy=A*y+B*(u+DD);