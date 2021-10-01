function dx = trig_dyn_toPeak(in1,udummy)
%TRIG_DYN_TOPEAK
%    DX = TRIG_DYN_TOPEAK(IN1,UDUMMY)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    01-Oct-2021 10:52:00

cq = in1(1,:);
q_dot_0 = in1(4,:);
q_ddot = in1(3,:);
sq = in1(2,:);
t = in1(5,:);
t2 = q_ddot.*t;
t3 = q_dot_0+t2;
dx = [-sq.*t3;cq.*t3;0.0;0.0;1.0];
