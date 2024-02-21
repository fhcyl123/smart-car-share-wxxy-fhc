%Sugeno type fuzzy control for single inverted pendulum
close all;

P=[-10-10i;-10+10i];     %Stable pole point

F1=place(A1,B1,P)
F2=place(A2,B2,P)
F3=place(A3,B3,P)
F4=place(A4,B4,P)
F5=place(A5,B5,P)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tc=newfis('tc','sugeno');
tc=addvar(tc,'input','theta',[-15,15]*pi/180);
tc=addmf(tc,'input',1,'NG','gaussmf',[5,-15]*pi/180);
tc=addmf(tc,'input',1,'ZR','gaussmf',[5,0]*pi/180);
tc=addmf(tc,'input',1,'PO','gaussmf',[5,15]*pi/180);

tc=addvar(tc,'input','omega',[-200,200]*pi/180);
tc=addmf(tc,'input',2,'NG','gaussmf',[50,-200]*pi/180);
tc=addmf(tc,'input',2,'ZR','gaussmf',[50,0]*pi/180);
tc=addmf(tc,'input',2,'PO','gaussmf',[50,200]*pi/180);

tc=addvar(tc,'output','u',[-300,0]);
tc=addmf(tc,'output',1,'No.1','linear',[F1(1),F1(2)]);
tc=addmf(tc,'output',1,'No.2','linear',[F2(1),F2(2)]);
tc=addmf(tc,'output',1,'No.3','linear',[F3(1),F3(2)]);
tc=addmf(tc,'output',1,'No.4','linear',[F4(1),F4(2)]);
tc=addmf(tc,'output',1,'No.5','linear',[F5(1),F5(2)]);

rulelist=[1 1 4 1 1;
          1 2 3 1 1;
          1 3 5 1 1;
          2 1 2 1 1;
          2 2 1 1 1;
          3 1 5 1 1;
          3 2 3 1 1;
          3 3 4 1 1];
tc=addrule(tc,rulelist);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model=newfis('model','sugeno');
model=addvar(model,'input','theta',[-15,15]*pi/180);
model=addmf(model,'input',1,'NG','gaussmf',[5,-15]*pi/180);
model=addmf(model,'input',1,'ZR','gaussmf',[5,0]*pi/180);
model=addmf(model,'input',1,'PO','gaussmf',[5,15]*pi/180);

model=addvar(model,'input','omega',[-200,200]*pi/180);
model=addmf(model,'input',2,'NG','gaussmf',[50,-200]*pi/180);
model=addmf(model,'input',2,'ZR','gaussmf',[50,0]*pi/180);
model=addmf(model,'input',2,'PO','gaussmf',[50,200]*pi/180);

model=addvar(model,'input','u',[-5,5]);
model=addmf(model,'input',3,'Any','gaussmf',[1.5,-5]);

model=addvar(model,'output','d_theta',[-200,200]*pi/180);
model=addmf(model,'output',1,'No.1','linear',[0 1 0 0]);
model=addmf(model,'output',1,'No.2','linear',[0 1 0 0]);
model=addmf(model,'output',1,'No.3','linear',[0 1 0 0]);
model=addmf(model,'output',1,'No.4','linear',[0 1 0 0]);
model=addmf(model,'output',1,'No.5','linear',[0 1 0 0]);

model=addvar(model,'output','d_omega',[-200,200]*pi/180);
model=addmf(model,'output',2,'No.1','linear',[A1(2,1),0,B1(2),0]);
model=addmf(model,'output',2,'No.2','linear',[A2(2,1),0,B2(2),0]);
model=addmf(model,'output',2,'No.3','linear',[A3(2,1),0,B3(2),0]);
model=addmf(model,'output',2,'No.4','linear',[A4(2,1),A4(2,2),B4(2),0]);
model=addmf(model,'output',2,'No.5','linear',[A5(2,1),A5(2,2),B5(2),0]);

rulelist1=[1 1 0 4 4 1 1;
           1 2 0 3 3 1 1;
           1 3 0 5 5 1 1;
           2 1 0 2 2 1 1;
           2 2 0 1 1 1 1;
           2 3 0 2 2 1 1;
           3 1 0 5 5 1 1;
           3 2 0 3 3 1 1;
           3 3 0 4 4 1 1];
model=addrule(model,rulelist1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ts=0.020;
x=[0.20;0];  %Initial state

for k=1:1:100
   time(k)=k*ts;
   u(k)=(-1)*evalfis([x(1),x(2)],tc);   %Using feedback control
   
   k0=evalfis([x(1),x(2),u(k)],model)'; %Using fuzzy T-S model
   x=x+ts*k0;
   
   y1(k)=x(1);
   y2(k)=x(2);
end
figure(1);
subplot(211);
plot(time,y1),grid on;
xlabel('time(s)'),ylabel('Angle');
subplot(212);
plot(time,y2),grid on;
xlabel('time(s)'),ylabel('Angle rate');

figure(2);
plot(time,u),grid on;
xlabel('time(s)'),ylabel('controller output');

figure(3);
plotmf(tc,'input',1);
figure(4);
plotmf(tc,'input',2);

showrule(tc);
showrule(model);