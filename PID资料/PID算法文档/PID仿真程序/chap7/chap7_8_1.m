%Zero Phase Error Frequency testing
clear all;
close all;

ts=0.001;
sys=tf(5.235e005,[1,87.35,1.047e004,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

kp=0.70;
error_1=0;
kk=0;       %Frequency steps 

for F=0.5:0.5:8
kk=kk+1;
FF(kk)=F;
   
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;
for k=1:1:2000
time(k)=k*ts;

%Tracing Sine high frequency Signal
rin(k)=0.5*sin(1*2*pi*F*k*ts);

%Linear model
yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;

error(k)=rin(k)-yout(k);
u(k)=kp*error(k);    %P Controller

%------------Return of PID parameters-------------%
u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
end

plot(time,rin,'r',time,yout,'b');
pause(0.0000000000001);

Y=rin(1001:1:2000)';

for i=1:1:1000
    fai(1,i) = sin(2*pi*F*i*ts);   
    fai(2,i) = cos(2*pi*F*i*ts);
end

c = inv(fai*fai')*fai*Y;

pinA = sqrt(c(1)*c(1)+c(2)*c(2));
pinF = atan(c(2)/c(1));

Y=yout(1001:1:2000)';

for i=1:1:1000
    fai(1,i) = sin(2*pi*F*i*ts);   
    fai(2,i) = cos(2*pi*F*i*ts);
end

c=inv(fai*fai')*fai*Y;

poutA = sqrt(c(1)*c(1)+c(2)*c(2));
poutF = atan(c(2)/c(1));

mag(kk)=20*log10(poutA/pinA);    %Magnitude
ph(kk)=(poutF-pinF)*180/pi;      %Phase error

end
FF=FF'
mag=mag'
ph=ph'

save freq.mat FF mag ph;
save closed.mat kp;