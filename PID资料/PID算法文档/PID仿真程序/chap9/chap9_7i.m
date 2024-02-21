%Flight Simulator Servo System
clear all;
close all;

J=2;
b=0.5;

kv=2;
kp=15;
kd=6;

f1=(b+kd*kv);
f2=J;

F=1;
A=1;
t=[0:0.001:10]';   %Simulation time

r=A*sin(2*pi*F*t);
dr=2*pi*F*A*cos(2*pi*F*t);
ddr=-4*pi*pi*F*F*A*sin(2*pi*F*t);