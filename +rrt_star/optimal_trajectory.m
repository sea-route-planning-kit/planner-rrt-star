function [trajectory, c_star, success] = optimal_trajectory(ship, R, x0, x1)
    
    R_inv = inv(R);
    [A,B] = ship.linearize(x1);

    % Example override
    %x0 = [5;0];
    %x1 = [6;2];
    %A = [ 0 1;
    %      0 0 ];
    %B = [ 0; 1];
    %R = 0.01;
    %R_inv = inv(R);


    %A   = [1 2 ; 3 4];
    %Bs  = [2*s; s^2];

    %R   = int(expm(A*s)*Bs,s,1,t);

    %G = @(t) (integral(@(tau) expm(A*(t-tau))*B*R_inv*B.'*expm(A*(t-tau)), 0, t));
    %%


    %%
    G = @(t) rrt_star.G(A,B,R_inv,t);
    %%
    %syms tau
    %G = @(t) double(int(expm(A*(t-tau))*B*R_inv*B.'*expm(A.'*(t-tau)), [0, t]));
    %%
    %G(10)
    %G(11)

    x_bar = @(t) expm(A*t)*x0;

    d = @(tau) G(tau)\(x1-x_bar(tau));

    %c_der = @(tau) 1 - 2*(A*x1).'*d(tau) - d(tau).'*B*R_inv*B.'*d(tau);
    c = @(tau) tau + (x1-x_bar(tau)).'*inv(G(tau))*(x1-x_bar(tau));
    %c_der = @(tau) rrt_star.c_der(A,B,R_inv,x1, d,tau);
    %tau_star = fzero(c_der, 50, struct('Display', 'iter'))
    [tau_star,c_star] = fmincon(c, 0, [],[],[],[], 0, 10000, [], struct('Display', 'iter'))
    %tau_star = 5;
    success = tau_star < 10000;
    
    if (success) 

        z = @(t) expm([ A B*R_inv*B.'; zeros(size(A)) -A.' ]*(t-tau_star)) * [x1; d(tau_star)];


        Ns = 50;
        x_data = zeros(Ns+1, length(A));
        u_data = zeros(Ns+1, size(B,2));
        for i=1:(Ns+1)
            t = (i-1)*tau_star/Ns;
            z_t = z(t);
            x_data(i,:) = z_t(1:length(A));
            y = z_t((length(A)+1):length(A)*2);
            u_data(i,:) = R_inv*B.'*y;
        end
        trajectory = iddata(x_data, u_data, tau_star/Ns);
        %plot(trajectory);
    end
end

