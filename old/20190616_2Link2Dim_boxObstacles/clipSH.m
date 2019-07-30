function [Vout, Eout] = clipSH(V, E, A, b);
%testing Sutherland Hodgman algorithm for clipping cubes with hyperplanes

dim = size(V, 2);

testV = A*V' - b;

% if testV is positive, then it's "behind" the plane...
% intersect the edges corresponding to these points with the plane.
Vidx = find(testV > 0);

% start constructing output list of vertices, edges
Vidx_good = find(testV <= 0);
if isempty(Vidx_good)
    % no intersection possible
    Vout = [];
    Eout = [];
    return;
elseif length(Vidx_good) == length(testV)
    % all vertices are good
    Vout = V;
    Eout = E;
end
Vout = V(Vidx_good, :);

Eidx_good = zeros(size(E, 1), 2);
for i = 1:length(Vidx_good)
    Eidx_good = Eidx_good + (E == Vidx_good(i));
end
Eidx_good = Eidx_good(:, 1) & Eidx_good(:, 2);
Eout = E(Eidx_good, :);
for i = 1:length(Vidx_good)
    Eout(Eout == Vidx_good(i)) = i;
end

for i = 1:length(Vidx)
    [Erow, Ecol] = find(E == Vidx(i)); % get edge corresponding to each vertex
    for j = 1:length(Erow)
        Etemp = E(Erow(j), :);
        %reorder edges so that "in front" vertex comes first
        if Ecol(j) == 1
            Etemp = fliplr(Etemp);
        end
        % compute intersection of plane with each edge...
        % say x = \lambda x_1 + (1 - \lambda) x_2
        % want to solve Ax - b = 0 for \lambda
        x1 = V(Etemp(1,1), :)';
        x2 = V(Etemp(1,2), :)';
        lambda = (b - A*x2)/(A*(x1 - x2));
        if lambda >= 0 && lambda <= 1 % intersection is possible
            newvert = (lambda*x1 + (1 - lambda)*x2)';
            if ~ismember(newvert, Vout, 'rows')
                % add new vertex
                Vout = [Vout; newvert];
                % add new edge
                Eout = [Eout; [size(Vout, 1), find(Vidx_good == Etemp(1,1))]];
            end
        end
    end
end

% add new edges
len = size(Vout, 1) - length(Vidx_good);
Vadd = Vout(end - len + 1:end, :);
% for i = 1:len - 1
%     for j = i+1:len
%         if sum(Vadd(i, :) == Vadd(j, :)) == (dim-1) % if the vertices share a face
%             Eout = [Eout; [len + i, len + j]];
%         end
%     end
% end

% project added vertices onto plane... take the convex hull
% get unit vectors in plane...
if ~isempty(Vadd)
    if dim >= 3
        U = orth(null(A));
        try
            K = convhulln(Vadd*U, {'Qt', 'Qx', 'Pp'});
            newK = [];
            Kcols = size(K, 2);
            for i = 1:Kcols - 1
                newK = [newK; K(:, i:i+1)];
            end
            newK = [newK; K(:, [1, Kcols])];
            newK = unique(newK, 'rows');
            len2 = length(Vidx_good);
            %             for i = 1:length(K)-1
            %                 Eout = [Eout; [len2 + K(i), len2 + K(i+1)]];
            %             end
            for i = 1:size(newK,1)
                Eout = [Eout; [len2 + newK(i, 1), len2 + newK(i, 2)]];
            end
        catch
            disp('Trouble with the convex hull code...probably a simplex of zero volume');
            len2 = length(Vidx_good);
            % just put an edge between any two vertices
            for i = 1:len-1
                for j = i:len
                    Eout = [Eout; len2+i, len2+j];
                end
            end
        end
    end
elseif dim == 2
    % just put an edge between the two vertices
    len2 = length(Vidx_good);
    Eout = [Eout; [len2 + 1, len2 + 2]];
end


end