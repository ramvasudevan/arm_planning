function d_joint_pos = joint_pos_LagrangeRemainder(c,s)

c1 = c(1);
c2 = c(2);
c3 = c(3);
c4 = c(4);
c5 = c(5);
c6 = c(6);
s1 = s(1);
s2 = s(2);
s3 = s(3);
s4 = s(4);
s5 = s(5);
s6 = s(6);

t2 = c2.*(4.4e+1./1.25e+2);
t3 = c2.*c4.*3.215e-1;
t4 = s3.*s4.*3.215e-1;
t5 = c3.*s2.*s4.*3.215e-1;
t6 = -t5;
t7 = t2+t3+t6+1.17e+2./1.0e+3;
mt1 = [t7,t4,0.0,c1.*(4.4e+1./1.25e+2)+c1.*c4.*3.215e-1,s1.*(4.4e+1./1.25e+2)+c4.*s1.*3.215e-1,c3.*s4.*(-3.215e-1),c1.*s2.*s4.*(-3.215e-1),s1.*s2.*s4.*(-3.215e-1),c2.*s4.*(-3.215e-1),c1.*c2.*3.215e-1,c2.*s1.*3.215e-1,s2.*(-3.215e-1),0.0,0.0,0.0,0.0,0.0,0.0,-t4,t7,0.0,c1.*c3.*s4.*(-3.215e-1),c3.*s1.*s4.*(-3.215e-1),c4.*(-3.215e-1)-4.4e+1./1.25e+2];
mt2 = [s1.*s4.*(-3.215e-1),c1.*s4.*3.215e-1,0.0,s1.*s3.*(-3.215e-1)-c1.*c3.*s2.*3.215e-1,c1.*s3.*3.215e-1-c3.*s1.*s2.*3.215e-1,c2.*c3.*(-3.215e-1),0.0,0.0,0.0,0.0,0.0,0.0];
d_joint_pos = reshape([mt1,mt2],3,12);
