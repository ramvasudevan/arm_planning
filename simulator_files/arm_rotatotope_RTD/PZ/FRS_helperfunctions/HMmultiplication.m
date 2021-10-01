function res = HMmultiplication(a,b)
% the multiplication of two homogenius transformation matrix
% each entry of the matrix is a polynomial zonotope.
% a and b are stored in 12-dim polynomial zonotope.

if isa(a, 'polyZonotope') || isa(a, 'zonotope')
    situation = 2;
    a1 = getDim(a,1);
    a2 = getDim(a,2);
    a3 = getDim(a,3);
    a4 = getDim(a,4);
    a5 = getDim(a,5);
    a6 = getDim(a,6);
    a7 = getDim(a,7);
    a8 = getDim(a,8);
    a9 = getDim(a,9);
    a10 = getDim(a,10);
    a11 = getDim(a,11);
    a12 = getDim(a,12);
else
    situation = 1;
    if isa(a, 'double')
        a1 = a(1);
        a2 = a(2);
        a3 = a(3);
        a4 = a(4);
        a5 = a(5);
        a6 = a(6);
        a7 = a(7);
        a8 = a(8);
        a9 = a(9);
        a10 = a(10);
        a11 = a(11);
        a12 = a(12);
    else
        a1 = a{1};
        a2 = a{2};
        a3 = a{3};
        a4 = a{4};
        a5 = a{5};
        a6 = a{6};
        a7 = a{7};
        a8 = a{8};
        a9 = a{9};
        a10 = a{10};
        a11 = a{11};
        a12 = a{12};
    end
end

if isa(b, 'polyZonotope') || isa(b, 'zonotope')
    b1 = getDim(b,1);
    b2 = getDim(b,2);
    b3 = getDim(b,3);
    b4 = getDim(b,4);
    b5 = getDim(b,5);
    b6 = getDim(b,6);
    b7 = getDim(b,7);
    b8 = getDim(b,8);
    b9 = getDim(b,9);
    b10 = getDim(b,10);
    b11 = getDim(b,11);
    b12 = getDim(b,12);
else
    if isa(b, 'double')
        b1 = b(1);
        b2 = b(2);
        b3 = b(3);
        b4 = b(4);
        b5 = b(5);
        b6 = b(6);
        b7 = b(7);
        b8 = b(8);
        b9 = b(9);
        b10 = b(10);
        b11 = b(11);
        b12 = b(12);
    else
        b1 = b{1};
        b2 = b{2};
        b3 = b{3};
        b4 = b{4};
        b5 = b{5};
        b6 = b{6};
        b7 = b{7};
        b8 = b{8};
        b9 = b{9};
        b10 = b{10};
        b11 = b{11};
        b12 = b{12};
    end
end

c = cell(12,1);

if situation == 1
    c{1} = a1*b1 + a4*b2 + a7*b3;
    c{2} = a2*b1 + a5*b2 + a8*b3;
    c{3} = a3*b1 + a6*b2 + a9*b3;
    c{4} = a1*b4 + a4*b5 + a7*b6;
    c{5} = a2*b4 + a5*b5 + a8*b6;
    c{6} = a3*b4 + a6*b5 + a9*b6;
    c{7} = a1*b7 + a4*b8 + a7*b9;
    c{8} = a2*b7 + a5*b8 + a8*b9;
    c{9} = a3*b7 + a6*b8 + a9*b9;
    c{10} = a10 + a1*b10 + a4*b11 + a7*b12;
    c{11} = a11 + a2*b10 + a5*b11 + a8*b12;
    c{12} = a12 + a3*b10 + a6*b11 + a9*b12;
else
    c{1} = exactPlus(exactPlus(a1*b1, a4*b2), a7*b3);
    c{2} = exactPlus(exactPlus(a2*b1, a5*b2), a8*b3);
    c{3} = exactPlus(exactPlus(a3*b1, a6*b2), a9*b3);
    c{4} = exactPlus(exactPlus(a1*b4, a4*b5), a7*b6);
    c{5} = exactPlus(exactPlus(a2*b4, a5*b5), a8*b6);
    c{6} = exactPlus(exactPlus(a3*b4, a6*b5), a9*b6);
    c{7} = exactPlus(exactPlus(a1*b7, a4*b8), a7*b9);
    c{8} = exactPlus(exactPlus(a2*b7, a5*b8), a8*b9);
    c{9} = exactPlus(exactPlus(a3*b7, a6*b8), a9*b9);
    c{10} = exactPlus(exactPlus(exactPlus(a10, a1*b10), a4*b11), a7*b12);
    c{11} = exactPlus(exactPlus(exactPlus(a11, a2*b10), a5*b11), a8*b12);
    c{12} = exactPlus(exactPlus(exactPlus(a12, a3*b10), a6*b11), a9*b12);
end

res = c{1};

for i = 2:12
    if isa(res, 'polyZonotope') || isa(c{i}, 'polyZonotope')
        res = exactCartProd(res, c{i});
    else
        if isa(res, 'zonotope') || isa(c{i}, 'zonotope')
            res = cartProd(res, c{i});
        else
            if isa(res, 'double') || isa(c{i}, 'double')
                res = [res; c{i}];
            else
                res = c;
                return;
            end
        end
    end
end

end

