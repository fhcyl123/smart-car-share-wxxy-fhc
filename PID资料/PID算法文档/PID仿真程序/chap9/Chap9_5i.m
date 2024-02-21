%Three Loop of Flight Simulator Servo System with Direct Current Motor
clear all;
close all;
%(1)Current loop
L=0.001;   %L<<1 Inductance of motor armature
R=1;       %Resistence of motor armature
ki=0.001;  %Current feedback coefficient

%(2)Velocity loop
kd=6;      %Velocity loop amplifier coefficient
kv=2;      %Velocity loop feedback coefficient

J=2;       %Equivalent moment of inertia of frame and motor
b=1;       %Viscosity damp coefficient of frame and motor

km=1.0;    %Motor moment coefficient
Ce=0.001;  %Voltage feedback coefficient

%Friction model: Coulomb&Viscous Friction
Fc=100.0;bc=30.0;  %Practical friction

%(3)Position loop: PID controller
ku=11;     %Voltage amplifier coefficient of PWM
kpp=150;
kii=0.1;
kdd=1.5;

%Friction Model compensation
%Equavalent gain from feedforward to practical friction
Gain=ku*kd*1/R*km*1.0;    
Fc1=Fc/Gain;	bc1=bc/Gain; %Feedforward compensation

%Input signal initialize
F=0.50;
A=0.50;
ts=0.001;   %Sampling time

M=2;
if M==1       %Sine Signal
	k=5000;
	time=[0:ts:k*ts]';   %Simulation time
	rin=A*sin(2*pi*F*time);
	drin=2*pi*F*A*cos(2*pi*F*time);
elseif M==2   %Random Signal
	T=4999;
	time=zeros(T,1);
	rin=zeros(T,1);
	drin=zeros(T,1);
 	rin(1)=0;
	drin(1)=0;
for k=1:1:T
	time(k+1)=k*ts;   
%Random signal
   rin(k+1)=A*sin(2*pi*F*k*ts)+0.5*A*sin(2*pi*0.5*F*k*ts)+...
            0.25*A*sin(2*pi*0.25*F*k*ts);
   drin(k+1)=(rin(k+1)-rin(k))/ts;
end
end