clear all;
close all;
nl_pid0=[0 0 0];
options=[1 0.01 0.01];
nl_pid=lsqnonlin('chap7_4f1',nl_pid0,options)