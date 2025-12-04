function beta = DCMToBeta(C)

    % Step 1: compute β_i^2 candidates
    b0_sq = 0.25 * (1 + trace(C));
    b1_sq = 0.25 * (1 + 2*C(1,1) - trace(C));
    b2_sq = 0.25 * (1 + 2*C(2,2) - trace(C));
    b3_sq = 0.25 * (1 + 2*C(3,3) - trace(C));

    b_sq = [b0_sq, b1_sq, b2_sq, b3_sq];

    % Find the index of the largest β_i
    [~, iMax] = max(b_sq);
    bMax = sqrt(b_sq(iMax));

    % Step 2: cross-product terms (exactly as slide)
    b0b1 = (C(2,3) - C(3,2)) * 0.25;
    b0b2 = (C(3,1) - C(1,3)) * 0.25;
    b0b3 = (C(1,2) - C(2,1)) * 0.25;

    b1b2 = (C(1,2) + C(2,1)) * 0.25;
    b3b1 = (C(3,1) + C(1,3)) * 0.25;
    b2b3 = (C(2,3) + C(3,2)) * 0.25;

    % Step 3: reconstruct the quaternion based on which β is largest
    switch iMax
        case 1        % β0 is largest
            beta0 = bMax;
            beta1 = b0b1 / beta0;
            beta2 = b0b2 / beta0;
            beta3 = b0b3 / beta0;

        case 2        % β1 is largest
            beta1 = bMax;
            beta0 = b0b1 / beta1;
            beta2 = b1b2 / beta1;
            beta3 = b3b1 / beta1;

        case 3        % β2 is largest
            beta2 = bMax;
            beta0 = b0b2 / beta2;
            beta1 = b1b2 / beta2;
            beta3 = b2b3 / beta2;

        case 4        % β3 is largest
            beta3 = bMax;
            beta0 = b0b3 / beta3;
            beta1 = b3b1 / beta3;
            beta2 = b2b3 / beta3;
    end

    beta = [beta0, beta1, beta2, beta3];

    % Normalize (protects against numerical drift)
    beta = beta / norm(beta);

    % Ensure short rotation (β₀ ≥ 0)
    if beta(1) < 0
        beta = -beta;
    end
end
