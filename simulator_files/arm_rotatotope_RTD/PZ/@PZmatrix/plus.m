function res = plus(inp1, inp2)
% addition between two PZmatrix

if isa(inp1,'double')
    res = inp2;
    res.gen(1) = res.gen(1) + inp1;
    return;
end

if isa(inp2,'double')
    res = inp1;
    res.gen(1) = res.gen(1) + inp2;
    return;
end

ifHaveMerged2 = false(1,62);

res = inp1;

%% center
res.gen(1) = res.gen(1) + inp2.gen(1);

%% merge redundant monomials
for i = 1:62
    p1 = 1:62;
    p2 = p1 + i - 1;
    p2(p2 > 62) = p2(p2 > 62) - 62;
    p1 = p1 + 1;
    p2 = p2 + 1;
    for j = 1:62 % parforparforparforparforparforparforparforparforparforparfor
        if inp1.gen(p1(j)) ~= 0 && inp2.gen(p2(j)) ~= 0 && inp1.deg(p1(j)) == inp2.deg(p2(j))
            res.gen(p1(j)) = inp1.gen(p1(j)) + inp2.gen(p2(j));
            ifHaveMerged2(p2(j) - 1) = true;
        end
    end
end

%% find 62 largest monomials from the rest of them
reduceGrest = 0;

ifUpdateSmallest = true;
for i = 1:62
    if ~ifHaveMerged2(i) && inp2.gen(i + 1) ~= 0
        % replace with smallest monomial in inp1 (selection sort)
        if ifUpdateSmallest
            smallestGen = 1000000;
            smallestGenId = 0;
            for j = 1:62
                if abs(res.gen(j + 1)) < smallestGen
                    smallestGen = abs(res.gen(1,j + 1));
                    smallestGenId = j;
                end
            end
        end
        
        if abs(inp2.gen(i + 1)) > smallestGen
            reduceGrest = reduceGrest + smallestGen;
            res.gen(smallestGenId + 1) = inp2.gen(i + 1);
            res.deg(smallestGenId + 1) = inp2.deg(i + 1);
            ifUpdateSmallest = true;
        end
    end
end

%% new Grest
res.gen(end) = abs(inp1.gen(end)) + abs(inp2.gen(end)) + reduceGrest;

end

