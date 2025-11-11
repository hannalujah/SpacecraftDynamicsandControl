function DCM = betaToDCM(beta)
    beta0 = beta(1);
    beta1 = beta(2);
    beta2 = beta(3);
    beta3 = beta(4);

    
    DCM = [beta0^2 + beta1^2 - beta2^2 - beta3^2, 2*(beta1*beta2 + beta0*beta3), 2*(beta1*beta3 - beta0*beta2);
        2*(beta1*beta2 - beta0*beta3), beta0^2 - beta1^2 + beta2^2 - beta3^2, 2*(beta2*beta3 + beta0*beta1);
        2*(beta1*beta3 + beta0*beta2), 2*(beta2*beta3 - beta0*beta1), beta0^2 - beta1^2 - beta2^2 + beta3^2];

end