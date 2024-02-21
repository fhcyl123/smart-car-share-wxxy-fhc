%Local linearization for single inverted pendulum
clear all;
close all;

g=9.8;
m=2;
M=8;
l=0.5;
a=l/(m+M);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Equation 1:
a1=g/(4/3*l-a*m*l);
A1=[0 1;
   a1 0]
   
b1=-a/(4/3*l-a*m*l);
B1=[0;b1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Equation 2:
x2=200*pi/180;
a2=(g-a*m*l*x2^2)/(4/3*l-a*m*l);
A2=[0 1;
   a2 0]
   
b2=b1;
B2=[0;b2]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Equation 3:
%x1=-15*pi/180;
x1=15*pi/180;

a3=g/(4/3*l-a*m*l*(cos(x1))^2)
A3=[0 1;
   a3 0]
b3=-a*cos(x1)/(4/3*l-a*m*l*(cos(x1))^2)
B3=[0;b3]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Equation 4:
x1=15*pi/180;
x2=200*pi/180;

a41=g/(4/3*l-a*m*l*(cos(x1))^2)
a42=-a*m*l*x2*sin(2*x1)*0.5/(4/3*l-a*m*l*(cos(x1))^2)
A4=[0   1;
   a41 a42]
b4=b3;
B4=[0;b4]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Equation 5:
x1=-15*pi/180;
x2=200*pi/180;

a51=g/(4/3*l-a*m*l*(cos(x1))^2)
a52=-a*m*l*x2*sin(2*x1)*0.5/(4/3*l-a*m*l*(cos(x1))^2)
A5=[0  1;
   a51 a52]
b5=b3;
B5=[0;b5]