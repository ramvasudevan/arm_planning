clear; clc;

PZ_reachset = load('FRS_trig_PZ/trig_FRS_2.937.mat');
OZ_reachset = load('FRS_trig/trig_FRS_2.937.mat');

figure; hold on;

factor = 1.8*rand - 0.9;

for i = [1:5:length(PZ_reachset.Rcont(1).timeInterval.set),length(PZ_reachset.Rcont(1).timeInterval.set)]
    PZ_sliceRes = sliceFRS(scaleFRS(PZ_reachset.Rcont(1).timeInterval.set{i}, 2, -1, 1), factor);
    plot((PZ_sliceRes), [1,2], 'Color', [1,0,0], 'Filled', false);
    
    OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont(1).timeInterval.set{i}, 3, factor);
    plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);
end

for i = [1:5:length(PZ_reachset.Rcont(2).timeInterval.set),length(PZ_reachset.Rcont(2).timeInterval.set)]
    PZ_sliceRes = sliceFRS(scaleFRS(PZ_reachset.Rcont(2).timeInterval.set{i}, 2, -1, 1), factor);
    plot((PZ_sliceRes), [1,2], 'Color', [1,0,0], 'Filled', false);
    
    OZ_sliceRes = zonotope_slice(OZ_reachset.Rcont(2).timeInterval.set{i}, 3, factor);
    plot(OZ_sliceRes, [1,2], 'Color', [0,0,1], 'Filled', false);
end

axis equal
% axis([0,1.5,0,1.5]);