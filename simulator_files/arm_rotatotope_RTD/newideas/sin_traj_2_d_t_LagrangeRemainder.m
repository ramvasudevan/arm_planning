function out1 = sin_traj_2_d_t_LagrangeRemainder(k,p0,t,v0)
%sin_traj_2_d_t_LagrangeRemainder
%    OUT1 = sin_traj_2_d_t_LagrangeRemainder(K,P0,T,V0)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    07-Nov-2021 22:29:25

t2 = k.^2;
t3 = k.^3;
t5 = k.^5;
t6 = t.^2;
t7 = v0.*2.0;
t9 = -t;
t10 = t-1.0;
t11 = t.*v0.*-2.0;
t13 = v0./4.0;
t4 = t2.^2;
t8 = t.*t7;
t12 = t6.*v0;
t14 = -t13;
t15 = t6./2.0;
t17 = t7+t11;
t16 = -t12;
t18 = t9+t15+1.0./4.0;
t19 = t18.^2;
t20 = t18.^3;
t22 = p0+t8+t14+t16;
t21 = t19.^2;
t23 = sin(t22);
t24 = cos(t22);
out1 = t17.*t24-k.*t10.*t24+k.*t17.*t18.*t23-t2.*t10.*t18.*t23+(t3.*t10.*t19.*t24)./2.0+(t4.*t10.*t20.*t23)./6.0-(t5.*t10.*t21.*t24)./2.4e+1-(t2.*t17.*t19.*t24)./2.0-(t3.*t17.*t20.*t23)./6.0+(t4.*t17.*t21.*t24)./2.4e+1+(t5.*t17.*t18.^5.*t23)./1.2e+2;
