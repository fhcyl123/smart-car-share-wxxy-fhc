function f=pid_ncd_pg_eq(nl_pid)
assignin('base','kp',nl_pid(1));
assignin('base','ki',nl_pid(2));
assignin('base','kd',nl_pid(3));
opt=simset('solver','ode5');
[tout,xout,yout]=sim('chap7_4f2',[0 10],opt);
f=yout-1;
