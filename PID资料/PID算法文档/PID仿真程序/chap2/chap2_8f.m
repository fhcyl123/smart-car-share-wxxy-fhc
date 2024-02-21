%PID Controler Based on Ziegler-Nichols
clear all;
close all;

ts=0.25;
sys=tf(1,[10,2,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

axis('square'),zgrid('new');

figure(1);
rlocus(dsys);
[km,pole]=rlocfind(dsys)

wm=angle(pole(1))/ts;
kp=0.6*km
kd=kp*pi/(4*wm)
ki=kp*wm/pi

dsys_pid=kp+kd*tf([1,-1],[1,1],ts)+ki*tf([1,0],[1,-1],ts)*ts;
dsysc=dsys*dsys_pid;
figure(2);
rlocus(dsysc);
axis('square'),zgrid;