function res = exactPlus(a,b)
% exactPlus for double array
if isa(a,'double') && isa(b,'double')
    res = a + b;
else
    error('Wrong input');
end

end

