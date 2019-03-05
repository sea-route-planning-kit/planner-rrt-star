function G = G(A,B,R_inv, t)
    %syms tau
    Ns = 100;
    %td = 0:t/Ts:t;
    Gd = zeros(Ns+1,length(A),length(A));
    T = 0:t/Ns:t;
    for i = 1:(Ns+1)
        td = T(i);%(i-1)*t/Ns;
        Gd(i,:,:) = expm(A*(t-td))*B*R_inv*B.'*expm(A.'*(t-td));
    end
    G = squeeze(trapz(T,Gd));
    
    %G = double(int(expm(A*(t-tau))*B*R_inv*B.'*expm(A.'*(t-tau)), [0, t]));
end

