function c_der = c_der(A,B,R_inv,x1, d, tau)
    d_tau = d(tau);
    c_der = 1 - 2*(A*x1).'*d_tau - d_tau.'*B*R_inv*B.'*d_tau;
end

