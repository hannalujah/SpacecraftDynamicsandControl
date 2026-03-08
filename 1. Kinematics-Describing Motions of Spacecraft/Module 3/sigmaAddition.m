function sigmaTotal = sigmaAddition(sigma1,sigma2)
    norm2Sigma1 = norm(sigma1)^2;
    norm2Sigma2 = norm(sigma2)^2;


    den = 1 + (norm2Sigma1 * norm2Sigma2) - (2*dot(sigma1,sigma2));
    flag = 0;
    
    while (den < 0.01)
       if (flag == 0)
            sigma1 = sigmaToSigmaS(sigma1);
            norm2Sigma1 = norm(sigma1)^2;
            flag = 1;
       else
           sigma2 = sigmaToSigmaS(sigma2);
           norm2Sigma2 = norm(sigma2)^2;
           flag = 0;
       end
       
        den = 1 + (norm2Sigma1 * norm2Sigma2) - (2*dot(sigma1,sigma2));
    end
    
    sigmaTotal = (1/den) * ( ((1-norm2Sigma1)*sigma2) + ((1-norm2Sigma2)*sigma1) - (2*cross(sigma2,sigma1)) );
end