function sigmaS = sigmaToSigmaS(sigma)

    sigmaS = (-1/(norm(sigma)^2)) * sigma;

end