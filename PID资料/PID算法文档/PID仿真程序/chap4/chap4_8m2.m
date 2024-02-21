function [u]=chap4_8m2(u1,u2,u3,u4)
global s
persistent w x1 x2 x3 w_1 w_2 w_3 

N=300;
C=5;

if u3==0
   w=zeros(N+C,1);
   w_1=w;
   w_2=w;
   d_w=w;
end
  
alfa=0.04;

if s==1       %Step Signal
   Smin=0;
   Smax=1;
elseif s==2   %Square Wave Signal
   Smin=-0.5;
   Smax=0.5;
end

xite=0.1; 

dvi=(Smax-Smin)/(N-1);

for i=1:1:C                %C size
    v(i)=Smin;
end
for i=C+1:1:C+N            %N size
    v(i)=v(i-1)+dvi;
end
for i=N+C+1:1:N+2*C        %C size
    v(i)=Smax;
end

rin=u1;
for i=1:1:N+C
if rin>=v(i)&rin<=v(i+C)
   a(i)=1;
else
   a(i)=0;
end
end

errork=u2;

un=a*w_1;
up=u4;
u=up+un;                %Total control output

if u>=10
   u=10;
end
if u<=-10
   u=-10;
end   

d_w=a'*xite*up/C;       

w=w_1+d_w+alfa*(w_1-w_2);
w_3=w_2;
w_2=w_1;
w_1=w;