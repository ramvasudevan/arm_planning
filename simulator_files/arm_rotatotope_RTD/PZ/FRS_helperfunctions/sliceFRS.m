function [pZ, gradient] = sliceFRS(FRS, alpha, sliceDim)
% slice FRS at alpha. Assume that the slicable factors are always in the first several rows of expMat
% FRS should be a CORA polyZonotope
% alpha should be a vector range from -1 to 1
% sliceDim should be a sorted vector representing the dimension needed to be 'sliced'. 
%          The default value is 1:length(alpha) (slice over the first several dimensions)

pZ = FRS;

numFac = length(alpha);

if nargin < 3
    sliceDim = 1:numFac;
end

if isa(FRS, 'double')
    gradient = zeros(length(pZ),length(sliceDim));
    return;
end

% sliceDim = intersect(sliceDim, pZ.id);

new_slice_dim = [];
new_alpha = [];
map_slice_dim = [];
for i = 1:length(sliceDim)
    temp = find(FRS.id == sliceDim(i));
    if ~isempty(temp)
        new_slice_dim = [new_slice_dim, temp];
        new_alpha = [new_alpha; alpha(i)];
        map_slice_dim = [map_slice_dim, i];
    end
end
sliceDim = new_slice_dim;
alpha_length = length(alpha);
alpha = new_alpha;
unsliceDim = setdiff(1:size(pZ.expMat,1), sliceDim);

if isempty(sliceDim)
    pZ = FRS;
    return;
end

if nargout > 1
    gradient = zeros(size(pZ.G,1),alpha_length);
    for i = 1:length(sliceDim)
        diffDim = sliceDim(i);
        nnzCols = pZ.expMat(diffDim,:) ~= 0;
        diffExpMat = pZ.expMat(sliceDim,nnzCols);
        diffExpMat(i,:) = diffExpMat(i,:) - 1;
        gradient(:,map_slice_dim(i)) = sum(pZ.G(:,nnzCols) .* pZ.expMat(diffDim,nnzCols) .* prod(alpha .^ diffExpMat, 1), 2);
    end
end

if ~isempty(pZ.G)
    pZ.G = pZ.G .* prod(alpha .^ pZ.expMat(sliceDim,:), 1);
    pZ.expMat = pZ.expMat(unsliceDim,:);
end
pZ.id(sliceDim) = [];

unControlFacIdx = pZ.id > pZ.initialId(end);
unControlFacNum = sum(unControlFacIdx);
pZ.id(unControlFacIdx) = (1:unControlFacNum) + pZ.initialId(end);

pZ = deleteZeros(pZ);

% reduce to zonotope if factors are empty
if isa(pZ,'polyZonotope')
    if isempty(pZ.expMat)
        if isempty(pZ.Grest)
            pZ = pZ.c;
        else
            pZ = zonotope(pZ.c, pZ.Grest);
        end
    end
end
