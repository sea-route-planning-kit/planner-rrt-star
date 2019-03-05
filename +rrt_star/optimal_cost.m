function [c_star, tau_star] = optimal_cost(ship, R, x0, x1)
    
    R_inv = inv(R);
    [A,B] = ship.linearize(x1);

    G = @(t) rrt_star.G(A,B,R_inv,t);

    x_bar = @(t) expm(A*t)*x0;

    d = @(tau) G(tau)\(x1-x_bar(tau));

    c = @(tau) tau + (x1-x_bar(tau)).'*inv(G(tau))*(x1-x_bar(tau));
    
    [tau_star,c_star] = fmincon(c, 0, [],[],[],[], 0, 10000, [], struct('Display', 'iter'))

end

