function dx=PlantModel(t,x,flag,para)
global F
dx=zeros(4,1);

F=2;
switch F
case 1
   r1=pi/6;r2=pi/6;
   dr1=0;dr2=0;
   ddr1=0;ddr2=0;
case 2
   r1=sin(3*t);r2=cos(3*t);
   dr1=3*cos(3*t);dr2=-3*sin(3*t);
   ddr1=-3*3*sin(3*t);ddr2=-3*3*cos(3*t);
case 3
   r1=sign(sin(3*t));r2=sign(sin(3*t));
   dr1=0;dr2=0;
   ddr1=0;ddr2=0;
end
dr=[dr1;dr2];
ddr=[ddr1;ddr2];

e1=x(1)-r1;
de1=x(2)-dr1;
e2=x(3)-r2;
de2=x(4)-dr2;

kp1=3000;kv1=230;
kp2=2000;kv2=210;

D11=2.462;
D22=0.362;
D12=0.147;
D21=0.147;

M=[D11 D12*cos(x(1)-x(3));
	D21*cos(x(1)-x(3)) D22];
G=[D12*x(4)^2*sin(x(1)-x(3));
   -D12*x(2)^2*sin(x(1)-x(3))];
C=[-x(4)*sin(x(3)) -(x(2)+x(4))*sin(x(3));
    x(2)*sin(x(3))  0];
 
S=3;
switch S
case 1
   u1=-kp1*e1-kv1*de1;
   u2=-kp2*e2-kv2*de2;
case 2
   u1=-kp1*e1-kv1*de1+M(1,:)*ddr+C(1,:)*dr+G(1);
   u2=-kp2*e2-kv2*de2+M(2,:)*ddr+C(2,:)*dr+G(2);
case 3
	Md=[D11 D12*cos(r1-r2);
  		 D21*cos(r1-r2) D22];
	Gd=[D12*dr1^2*sin(r1-r2);
	   -D12*dr2^2*sin(r1-r2)];
   Cd=[-dr2*sin(r2) -(dr1+dr2)*sin(r2);
   	  dr1*sin(r2)  0];
   u1=-kp1*e1-kv1*de1+Md(1,:)*ddr+Cd(1,:)*dr+Gd(1);
   u2=-kp2*e2-kv2*de2+Md(2,:)*ddr+Cd(2,:)*dr+Gd(2);
end
tol=[u1;u2];

Q=inv(M)*(tol-C*[x(2);x(4)]-G);

dx(1)=x(2);
dx(2)=Q(1);
dx(3)=x(4);
dx(4)=Q(2);