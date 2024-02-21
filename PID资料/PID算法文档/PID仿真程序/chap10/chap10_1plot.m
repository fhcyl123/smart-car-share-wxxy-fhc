close all;

figure(1);
plot(t,e,'r');
xlabel('time(s)');ylabel('error');

figure(2);
plot(t,y(:,1),'r',t,y(:,2),'b');
xlabel('time(s)');ylabel('position tracking');

figure(3);
plot(t,dy(:,1),'r',t,dy(:,2),'b');
xlabel('time(s)');ylabel('speed tracking');

figure(4);
plot(t,tol,'r');
xlabel('time(s)');ylabel('tol');