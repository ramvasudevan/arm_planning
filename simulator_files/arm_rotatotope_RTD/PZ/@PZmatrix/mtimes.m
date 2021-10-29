function res = mtimes(inp1, inp2)
% multiplication between two PZmatrix

if isa(inp1,'double')
    res = inp2;
    res.gen = res.gen * inp1;
    return;
end

if isa(inp2,'double')
    res = inp1;
    res.gen = res.gen * inp2;
    return;
end

mulGen = zeros(63,63); % __shared__
mulDeg = uint32(zeros(63,63)); % __shared__
ifHaveMerged = false(63,63); % __shared__

%% perform multiplication
for i = 1:63 % parforparforparforparforparforparforparforparfor
    for j = 1:63 % parforparforparforparforparforparforparforparfor
        mulGen(i,j) = inp1.gen(i) * inp2.gen(j);
        mulDeg(i,j) = inp1.deg(i) + inp2.deg(j); % degree may overflow ...
    end
end

%% merge redundant monomials
for i = 1:63
    for j = 1:63
        if ~ifHaveMerged(i,j)
            for k = 1:63 % parforparforparforparforparforparforparforparfor
                if k ~= i
                    for p = 1:63 % parforparforparforparforparforparforparforparfor
                        if ~ifHaveMerged(k,p)
                            if mulGen(k,p) == 0
                                ifHaveMerged(k,p) = true;
                            else
                                if mulDeg(i,j) == mulDeg(k,p)
                                    mulGen(i,j) = mulGen(i,j) + mulGen(k,p);
                                    ifHaveMerged(k,p) = true;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

%% find 62 largest monomials from the rest of them (reduce number of monomials so that won't exceed memory limit)
res = PZmatrix;
res.numberOfDecisionVariable = inp1.numberOfDecisionVariable;
res.gen(1) = mulGen(1,1);

fillInNum = 1;
ifUpdateSmallest = true;
reduceGrest = 0;
for i = 2:3969
    if ~ifHaveMerged(i) && mulGen(i) ~= 0
        if fillInNum <= 62
            % initialize res with the first 62 monomials
            res.gen(fillInNum + 1) = mulGen(i);
            res.deg(fillInNum + 1) = mulDeg(i);
            fillInNum = fillInNum + 1;
        else
            % replace with smallest monomial in res (selection sort)
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

            if abs(mulGen(i)) > smallestGen
                reduceGrest = reduceGrest + smallestGen;
                res.gen(smallestGenId + 1) = mulGen(i);
                res.deg(smallestGenId + 1) = mulDeg(i);
                ifUpdateSmallest = true;
            end
        end
    end
end

res.gen(end) = abs(inp1.gen(end)) * sum(abs(inp2.gen(1:63))) + abs(inp2.gen(end)) * sum(abs(inp1.gen(1:63))) + abs(inp1.gen(end)) * abs(inp2.gen(end)) + reduceGrest;

