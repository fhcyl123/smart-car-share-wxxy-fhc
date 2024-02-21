%Zero Phase Error controller verify
clear all;
close all;
ts=0.001;
rin_5=0;rin_4=0;rin_3=0;rin_2=0;rin_1=0;
F=3;
G=2000;
for k=1:1:G
time(k)=k*ts;
rin(k)=0.50*sin(F*2*pi*k*ts);

M=2;
if M==1
	rin(k+3)=0.50*sin(F*2*pi*(k+3)*ts);
	rin(k+2)=0.50*sin(F*2*pi*(k+2)*ts);
	rin(k+1)=0.50*sin(F*2*pi*(k+1)*ts);
	yout(k)=-0.1328*rin(k+2)+1.266*rin(k+1)-0.1328*rin(k);
elseif M==2
	yout(k)=-0.1328*rin_1+1.266*rin_2-0.1328*rin_3;
end

rin_5=rin_4;rin_4=rin_3;rin_3=rin_2;rin_2=rin_1;rin_1=rin(k);
end
figure(1);
plot(time(1:1:G),rin(1:1:G),'k',time(1:1:G),yout(1:1:G),'k');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time(1:1:G),rin(1:1:G)-yout(1:1:G),'k');
xlabel('time(s)');ylabel('error');