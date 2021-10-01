function disp(obj)

maximumAllowedDegree = floor(32 / obj.numberOfDecisionVariable);
            
expMat = sparse(obj.numberOfDecisionVariable, 62);

mask = bitshift(1,maximumAllowedDegree) - 1;

fprintf('%.3f ', obj.gen(1));

for i = 1:62
    if obj.gen(i + 1) ~= 0
        if obj.gen(i + 1) > 0
            fprintf('+');
        end
        
        fprintf('%.3f ', obj.gen(i + 1));
        
        elt = uint32(obj.deg(i + 1));

        for j = obj.numberOfDecisionVariable:-1:1
            eltDegree = bitand(elt, mask);
            
            if eltDegree > 0
                fprintf(' * x%d ^ %d ', j, bitand(elt, mask));
            end
            
            elt = bitshift(elt, -maximumAllowedDegree);
        end
        fprintf('\n');
    end
end