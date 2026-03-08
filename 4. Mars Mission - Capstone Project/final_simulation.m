function [tHist, sigmaHist, wHist, uHist] = final_simulation(t0, tf, dt, ...
    sigma0, w0, K, P, I_dc)

    tHist = t0:dt:tf;
    N = length(tHist);

    sigmaHist = zeros(3,N);
    wHist = zeros(3,N);
    uHist = zeros(3,N);

    x = [sigma0; w0];

    sigmaHist(:,1) = sigma0;
    wHist(:,1) = w0;

    for k = 1:N-1
        t = tHist(k);

        [RN, w_RN_N, ~] = getReferenceFrame(t);

        sigma_BN = x(1:3);
        w_BN = x(4:6);

        [sigma_BR, w_BR] = attitudeErrorEval(sigma_BN, w_BN, RN, w_RN_N);

        u = -K * sigma_BR - P * w_BR;

        x = rk4_step(x, u, dt, I_dc);

        sigmaHist(:,k+1) = x(1:3);
        wHist(:,k+1) = x(4:6);
        uHist(:,k) = u;
    end
end