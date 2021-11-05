function cos_traj_2_poly = cos_traj_2_TaylorExpansion(k,p0,tmid,v0)
%COS_TRAJ_2_TAYLOREXPANSION
%    COS_TRAJ_2_POLY = COS_TRAJ_2_TAYLOREXPANSION(K,P0,TMID,V0)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    04-Nov-2021 22:41:50

t2 = tmid.^2;
t3 = tmid.*v0.*2.0;
t4 = -tmid;
t6 = v0./4.0;
t5 = t2.*v0;
t7 = -t6;
t8 = t2./2.0;
t9 = -t5;
t10 = t4+t8+1.0./4.0;
t11 = p0+t3+t7+t9;
t12 = sin(t11);
t13 = cos(t11);
cos_traj_2_poly = t13-(k.^2.*t10.^2.*t13)./2.0-(k.^3.*t10.^3.*t12)./6.0+(k.^4.*t10.^4.*t13)./2.4e+1+(k.^5.*t10.^5.*t12)./1.2e+2+k.*t10.*t12;
