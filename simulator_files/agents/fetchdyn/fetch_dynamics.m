function [H, C] = fetch_dynamics(q ,dq)
    % mass matrix, M in the MLS book
    H = fetchH(q);
    
    % rest of the terms
    C = fetchC(q,dq);
end

