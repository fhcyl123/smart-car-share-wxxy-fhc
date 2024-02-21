%Closed system approaching and zero phase error controller design
clear all;
close all;
load freq.mat;

ts=0.001;
f=FF;
for i=1:length(ph)
   if ph(i)>0  ph(i)=ph(i)-360;
   end
end

%Transfer function approaching
%(1)Freq parameters
w=2*pi*f;           %From Hz to rad/sec
mag1=10.^(mag/20);  %From dB to degree
ph1=ph*pi/180;      %From degree to radian

h=mag1.*cos(ph1)+j*mag1.*sin(ph1);

%(2)Continous function
na=3;   %Three ranks approaching
nb=1;
%bb and aa are real numerator and denominator of transfer function
[bb,aa]=invfreqs(h,w,nb,na);   %w contains the frequency values in radians/s
display('Transfer function approaching is:');
sysx=tf(bb,aa)
[zs,ps,ks]=zpkdata(sysx,'v');

%(3)Discrete function
Gc=c2d(sysx,ts,'zoh');
zpksys=zpk(Gc);
[z,p,k]=zpkdata(zpksys,'v');    %Getting zero-pole-gain: z,p,k
display('Zeros and Poles of the Transfer function is:');
z
p
%In z-1 format
zGc=tf(Gc);
[nGc,dGc]=tfdata(zGc,'v');
zGc=filt(nGc,dGc,ts);

%(4)Magnitude and Phase to draw Bode
%Frequency response:create the complex frequency response vector: h=a+bi
h=freqs(bb,aa,w);
%Magnitude and phase
sysmag=abs(h);                 %Degree
sysmag1=20*log10(sysmag);      %From degree to dB
sysph=angle(h);                %Get radian
sysph1=sysph*180/pi;           %From radian to degree

%(5)Drawing practical plant and its approach function Bode to compare
figure(1);
subplot(2,1,1); 
semilogx(w,sysmag1,'r',w,mag,'b');grid on;
xlabel('Frequency(rad/sec)');ylabel('magnitude approach(dB)');

subplot(2,1,2);
semilogx(w,sysph1,'r',w,ph,'b');grid on;
xlabel('Frequency(rad/sec)');ylabel('phase approach(deg)');

figure(2);
magError=sysmag1-mag;
phError=sysph1-ph;
plot(w,phError,'r',w,magError,'b');
xlabel('Frequency(rad/sec)');ylabel('error of phase(deg) and magnitude(dB)');

%(6)Plant in zero-pole-gain format
zu=z(1);   %Unstable zero point
z1=z(2);
p1=p;
p1(4)=1/zu;

k1=1;
Gctemp=zpk(z1,p1,k1,ts);
dc=dcgain(Gctemp);        %Getting DC gain
k1=1/dc;                  %G(1)=1;

Gcn=zpk(z1,p1,k1,ts);

%(7)Design controller
Fdz=1/Gcn;      %Fdz=zpk(p1,z1,1/k1,ts);   
display('ZPE Controller is:');
tfdz=tf(Fdz)    %z^(-3):three rank delay

[nn1,dd1]=tfdata(tfdz,'v');

nF=nn1;
dF(1)=dd1(4);   %z^(-3):three rank delay
dF(2)=dd1(5);
dF(3)=dd1(1);
dF(4)=dd1(2);
dF(5)=dd1(3);

format long;
display('ZPE Controller coefficient is:');
nF
dF
save zpecoeff.mat nF dF;

%Controller
F=filt(nF,dF,ts);   %Equal conversion:from z to z-1
dcgain(F);

%(8)Verify the controller
Gn=series(F,Gc);    %Gn(z)=F(z)*Gc(z)
figure(3);
bode(Gn);

yk=minreal(Gn,ts);  %Simpling Gn(z)