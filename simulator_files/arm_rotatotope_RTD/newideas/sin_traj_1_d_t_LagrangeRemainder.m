function out1 = sin_traj_1_d_t_LagrangeRemainder(k,p0,t,v0)
%sin_traj_1_d_t_LagrangeRemainder
%    OUT1 = sin_traj_1_d_t_LagrangeRemainder(K,P0,T,V0)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    04-Nov-2021 20:04:43

t2 = t.*v0;
t3 = p0+t2;
t4 = cos(t3);
out1 = t4.*v0+(k.*t.*t4)./2.0-(k.*t.*t2.*sin(t3))./4.0;
