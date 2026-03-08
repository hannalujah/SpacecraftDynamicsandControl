function DCM = betaToDCM(beta)
    beta0 = beta(1);
    beta1 = beta(2);
    beta2 = beta(3);
    beta3 = beta(4);
    
    beta02 = beta0^2;
    beta12 = beta1^2;
    beta22 = beta2^2;
    beta32 = beta3^2;

    
    DCM = [beta02 + beta12 - beta22 - beta32, 2*(beta1*beta2 + beta0*beta3), 2*(beta1*beta3 - beta0*beta2);
        2*(beta1*beta2 - beta0*beta3), beta02 - beta12 + beta22 - beta32, 2*(beta2*beta3 + beta0*beta1);
        2*(beta1*beta3 + beta0*beta2), 2*(beta2*beta3 - beta0*beta1), beta02 - beta12 - beta22 + beta32];

end