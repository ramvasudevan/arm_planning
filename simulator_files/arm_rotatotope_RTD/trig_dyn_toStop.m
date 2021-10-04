function dx = trig_dyn_toStop(in1,udummy)
%TRIG_DYN_TOSTOP
%    DX = TRIG_DYN_TOSTOP(IN1,UDUMMY)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    04-Oct-2021 00:30:30

cq = in1(1,:);
q_dot_0 = in1(4,:);
q_ddot = in1(3,:);
sq = in1(2,:);
t = in1(5,:);
t2 = q_dot_0.*2.0;
t4 = q_ddot./2.0;
t5 = t-1.0./2.0;
t3 = q_ddot+t2;
t6 = t3.*t5;
t7 = -t6;
t8 = q_dot_0+t4+t7;
dx = [-sq.*t8;cq.*t8;0.0;0.0;1.0];
