function x = cubelet_ND(dim)
% x = cubelet
% 8-by-3 array of vertices of a unit cube.

x = zeros(2^dim, dim);
for j = 1:dim
    for i = 1:2^dim
        x(i, j) = (-1)^(floor( (i-1)/(2^(j-1))) +1);
    end
end

end