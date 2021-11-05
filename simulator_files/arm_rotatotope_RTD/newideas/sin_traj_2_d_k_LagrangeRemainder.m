function out1 = sin_traj_2_d_k_LagrangeRemainder(k,p0,t,v0)
%SIN_TRAJ_2_D_K_LAGRANGEREMAINDER
%    OUT1 = SIN_TRAJ_2_D_K_LAGRANGEREMAINDER(K,P0,T,V0)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    04-Nov-2021 22:41:51

t2 = t.^2;
out1 = (sin(k./4.0-p0+v0./4.0-t.*(k+v0.*2.0)+t2.*(k./2.0+v0)).*(-t+t2./2.0+1.0./4.0).^6)./7.2e+2;
