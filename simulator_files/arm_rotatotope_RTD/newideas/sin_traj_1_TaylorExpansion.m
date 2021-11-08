function sin_traj_1_poly = sin_traj_1_TaylorExpansion(k,p0,tmid,v0)
%sin_traj_1_TaylorExpansion
%    SIN_TRAJ_1_POLY = sin_traj_1_TaylorExpansion(K,P0,TMID,V0)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    07-Nov-2021 22:29:24

t2 = tmid.*v0;
t3 = p0+t2;
t4 = cos(t3);
t5 = sin(t3);
sin_traj_1_poly = t5+(k.*t4.*tmid.^2)./2.0-(k.^2.*t5.*tmid.^4)./8.0-(k.^3.*t4.*tmid.^6)./4.8e+1+(k.^4.*t5.*tmid.^8)./3.84e+2+(k.^5.*t4.*tmid.^10)./3.84e+3;
