function out1 = sin_traj_1_d_t_LagrangeRemainder(k,p0,t,v0)
%SIN_TRAJ_1_D_T_LAGRANGEREMAINDER
%    OUT1 = SIN_TRAJ_1_D_T_LAGRANGEREMAINDER(K,P0,T,V0)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    04-Nov-2021 22:41:50

t2 = t.*v0;
t3 = k.^2;
t4 = k.^3;
t6 = k.^5;
t5 = t3.^2;
t7 = p0+t2;
t8 = cos(t7);
t9 = sin(t7);
out1 = t8.*v0-(t.^3.*t3.*t9)./2.0-(t.^5.*t4.*t8)./8.0+(t.^7.*t5.*t9)./4.8e+1+(t.^9.*t6.*t8)./3.84e+2+k.*t.*t8-(k.*t.*t2.*t9)./2.0-(t.^3.*t2.*t3.*t8)./8.0+(t.^5.*t2.*t4.*t9)./4.8e+1+(t.^7.*t2.*t5.*t8)./3.84e+2-(t.^9.*t2.*t6.*t9)./3.84e+3;
