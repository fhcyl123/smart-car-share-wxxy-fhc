%PID control with Anti-windup
clear all;
close all;

alfa=1.0;

kp=50;
ki=10;
kd=1;

umin=0;
umax=10;

ua=(umin+umax)/2;