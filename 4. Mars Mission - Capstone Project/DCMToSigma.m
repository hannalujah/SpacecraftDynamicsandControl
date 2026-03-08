function sigma = DCMToSigma(C)
zeta = sqrt(1 + trace(C));

sigma = (1/(zeta*(zeta+2))) * ...
       [C(2,3)-C(3,2);
        C(3,1)-C(1,3);
        C(1,2)-C(2,1)];
    
if (norm(sigma) > 1)
   sigma = sigmaToSigmaS(sigma);
end
end