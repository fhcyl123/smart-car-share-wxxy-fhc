function [y]=func(x1,x2,x3)

for l1=1:1:3
   gs1=-[(x1+pi/6-(l1-1)*pi/6)/(pi/12)]^2;
	u1(l1)=exp(gs1);
end

for l2=1:1:3
   gs2=-[(x2+pi/6-(l2-1)*pi/6)/(pi/12)]^2;
	u2(l2)=exp(gs2);
end
 
U=[-20 -10  0
   -10  0  10
    0  10  20];

fnum=0;
fden=0;
for i=1:1:3
	for j=1:1:3
		fnum=fnum+u1(i)*u2(j)*U(i,j);
		fden=fden+u1(i)*u2(j);
	end
end

y=fnum/(fden+0.01);