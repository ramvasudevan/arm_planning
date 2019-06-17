function A = hypercube(d)
% hypercube(d) is a logical adjacency matrix for a d-dimensional hypercube.
    pow2 = 2.^(0:d-1);
    f2b = @(j) floor(rem(j./pow2,2)); % flint to binary
    b2f = @(b) b*pow2'; % binary to flint
    n = 2^d;
    A = zeros(n,n,'logical');
    for j = 0:n-1
        % Convert column index to binary.
        b = f2b(j);
        % Flip bits to get corresponding row indices.
        for i = 1:d
            b(i) = ~b(i);
            k = b2f(b);
            b(i) = ~b(i);
            A(k+1,j+1) = 1;
        end
    end
end