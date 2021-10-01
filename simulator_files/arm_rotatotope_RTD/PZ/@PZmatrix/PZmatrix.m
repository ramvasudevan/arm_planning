classdef PZmatrix
    % a alternative representation for a CORA polynomial zonotope
    % write the code in a pesudo parallel way to prepare for cuda
    
    properties
        % the first number of gen is the center of polyZonotope
        % the last number of gen is Grest (1-dim error)
        % the rest of gen is G (generators)
        gen (1,64) {mustBeNumeric,mustBeFinite} = zeros(1,64); 
        deg (1,64) {mustBeInteger,mustBeNonnegative} = uint32(zeros(1,64));
        numberOfDecisionVariable (1,1) {mustBeInteger,mustBeNonnegative} = 0;
    end
    
    methods
        function obj = PZmatrix(PZ)
            if nargin == 0
                return;
            end
            
            if ~isa(PZ, 'polyZonotope')
                error('have to initialize PZmatrix using a pre-defined polyZonotope');
            end
            
            if length(PZ.c) ~= 1
                error('only support 1-dim polyZonotope');
            end
            
            if length(PZ.G) > 62
                error('number of generators in polyZonotope should be less than or equal to 62');
            end
            
            obj.gen(1) = PZ.c;
            obj.gen(end) = abs(PZ.Grest(1));
            obj.gen(2:(2+length(PZ.G)-1)) = PZ.G;
            
            obj.numberOfDecisionVariable = length(PZ.initialId);
            
            obj.deg(2:63) = obj.hashDegree(PZ.expMat, PZ.id);
        end
        
        function hashExpMat = hashDegree(obj, expMat, id)
            % We are not using discrete sparse numbers (an array) to represent degrees of a polynomial zonotope. 
            % Instead, we hash the degrees as an unsigned integer number.
            % We need the following assumptions on the degree of the polynomial zonotope:
            % 1. Suppose the number of initialId (number of decision variable to track, number of slicable dimensions)
            % times the maximum allowed degree is smaller than 32, so that the hashed
            % integer can be represented using a regular 4 byte unsigned int.
            % For Digit, we may want to use 4 initialId and maximum allowed degree is
            % 2^8 - 1 = 255
            % 2. Suppose length of expMat is less than or equal to 62.

            maximumAllowedDegree = floor(32 / obj.numberOfDecisionVariable);

            hashExpMat = uint32(zeros(1,62));

            completeExpMat = sparse(obj.numberOfDecisionVariable, size(expMat,2));
            
            for i = 1:length(id)
                completeExpMat(id(i),:) = expMat(i,:);
            end
            expMat = full(completeExpMat);

            for i = 1:size(expMat,2)
                elt = uint32(0);
                for j = 1:obj.numberOfDecisionVariable
                    elt = elt + uint32(expMat(j,i));

                    if j < obj.numberOfDecisionVariable
                        elt = bitshift(elt, maximumAllowedDegree);
                    end
                end
                hashExpMat(i) = elt;
            end
        end
        
        function PZ = toPolyZonotope(obj)
            % convert a PZ matrix back to a CORA polyZonotope
            maximumAllowedDegree = floor(32 / obj.numberOfDecisionVariable);
            
            expMat = sparse(obj.numberOfDecisionVariable, 62);
            
            mask = bitshift(1,maximumAllowedDegree) - 1;
            
            for i = 1:62
                elt = uint32(obj.deg(i + 1));
                
                for j = obj.numberOfDecisionVariable:-1:1
                    expMat(j,i) = bitand(elt, mask);
                    elt = bitshift(elt, -maximumAllowedDegree);
                end
            end
            
            PZ = polyZonotope(obj.gen(1), obj.gen(2:63), obj.gen(end), expMat);
            PZ = deleteZeros(PZ);
        end
    end
end

