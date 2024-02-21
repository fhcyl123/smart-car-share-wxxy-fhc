function [u]=pid_aw1f1(u1,u2,u3,u4)

e=u2;
un=u3;
us=u4;

M=2;
switch M
case 1           %PID
   u=e;
case 2           %Anti-windup PID
   umin=0;
   umax=10;
   ua=(umin+umax)/2;
   if un~=us&e*(un-ua)>0
	u=u1;
   else
   	u=e;
   end
end