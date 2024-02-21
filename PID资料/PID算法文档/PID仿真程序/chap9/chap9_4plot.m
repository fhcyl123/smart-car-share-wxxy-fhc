close all;

figure(1);
plot(t,y(:,1),'k',t,y(:,2),'k');
xlabel('time(s)');ylabel('Position tracking');

figure(2);
plot(Ff(:,1),Ff(:,2),'k');
xlabel('Angle speed');ylabel('Friction force');

figure(3);
plot(t,dy(:,1),'k',t,dy(:,2),'k');
xlabel('time(s)');ylabel('Speed tracking');