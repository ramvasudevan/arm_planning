% playing with orientations...
% trying to figure out end of 2nd fetch link position

syms q1 q2 q3 q4 L1 L2 real

Owa = make_orientation(q1, 3);
Oab = make_orientation(q2, 2);
Obc = make_orientation(q3, 1);
Ocd = make_orientation(q4, 2);

x = Owa*Oab*[L1; 0; 0] + Owa*Oab*Obc*Ocd*[L2; 0; 0];

% expr = Obc*Ocd == (Owa*Oab)';
% expr = Owa*Oab*Obc*Ocd == eye(3);
bleh = rand(3,1);
bleh = bleh./norm(bleh);
expr = Owa*Oab*Obc*Ocd*[1;0;0] == bleh;

% expr = subs(expr, [q1;q2], [pi/4; pi/4]);

sol = solve(expr, [q3;q4])