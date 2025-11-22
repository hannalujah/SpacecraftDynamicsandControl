function sigma = betaToSigma(beta)
    sigma_1 = [];    
    for i = 2:4
        sigma_1 = [sigma_1, beta(i)/(1+beta(1))];
    end
    
    sigma_2 = -sigma_1 / (norm(sigma_1)^2);
    
    if (norm(sigma_1) < 1)
        sigma = sigma_1;
    else
        sigma = sigma_2;
    end
   
end