function disp(obj)

% uninitialized
if obj.numberOfDecisionVariable == 0 
    fprintf("0\n");
    return;
end

maximumAllowedDegree = floor(32 / obj.numberOfDecisionVariable);

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

fprintf('+[-%f, %f]\n', obj.gen(end), obj.gen(end));