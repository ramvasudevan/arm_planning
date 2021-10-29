clear; clc;

load('0key.mat');
dt = 1.0 / 128;
g_IC = (c_IC(2) - c_IC(1)) / 2;

for i = 1:length(c_IC)
    filename = sprintf('trig_FRS_%0.3f.mat', c_IC(i));
    load(filename);
    
    cosG = [];
    sinG = [];
    vG = [];
    cosGrest = [];
    sinGrest = [];
    vGrest = [];
    
    for j = 1:length(Rcont)
        if ~isempty(Rcont{j}.G)
            cosG = [cosG, Rcont{j}.c(1), Rcont{j}.G(1,1)];
            sinG = [sinG, Rcont{j}.c(2), Rcont{j}.G(2,1)];
            
            cosGrest = [cosGrest, Rcont{j}.Grest(1,1) + sum(abs(Rcont{j}.G(1,2:end)))];
            sinGrest = [sinGrest, Rcont{j}.Grest(2,2) + sum(abs(Rcont{j}.G(2,2:end)))];
        else
            cosG = [cosG, Rcont{j}.c(1), 0];
            sinG = [sinG, Rcont{j}.c(2), 0];
            
            cosGrest = [cosGrest, Rcont{j}.Grest(1,1)];
            sinGrest = [sinGrest, Rcont{j}.Grest(2,2)];
        end
        
        tmid = (j - 0.5) * dt;
        v_pk = c_IC(i) + g_k * 0.5;
        if j <= length(Rcont) / 2
            vG = [vG, c_IC(i), g_k * tmid];
            vGrest = [vGrest, g_IC + g_k * (dt / 2)];
        else
            vG = [vG, 2 * c_IC(i) * (1 - tmid), g_k * (1 - tmid)];
            vGrest = [vGrest, g_IC * dt + g_k * (dt / 2)];
        end
    end
    
    save(['mex_',filename], 'cosG', 'sinG', 'cosGrest', 'sinGrest', 'vG', 'vGrest', 'options', 'L', 't_plan', 't_total', 'my_c_IC', 'g_k');
end