function dx=DynamicModel(t,x,flag,para)
dx=zeros(4,1);

v=x(2);
F=x(3);
G=x(4);

vd=30;

nmn1=0.01;
nmn2=0.01;

S1=1;
S2=1;

T=100*0.001;   %100ms
K=60;

I=-F*v+G*vd;

dx(1)=-1/T*x(1)+1/T*vd;
dvm=dx(1);
dx(2)=K*I;
dx(3)=nmn1/(2*S1)*(S1^2-F^2)*(-K^2*v^2*F+K^2*vd*v*G-K*dvm*v);
dx(4)=nmn2/(2*S2)*(S2^2-G^2)*(-K^2*v*vd*F-K^2*vd^2*G+K*dvm*vd);