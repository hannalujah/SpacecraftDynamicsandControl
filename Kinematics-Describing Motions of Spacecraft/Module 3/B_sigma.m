function B = B_sigma(sigma)

    norm2Sigma = norm(sigma)^2;    
    B = ((1-norm2Sigma)*eye(3)) + (2*tilda(sigma)) + (2*sigma*transpose(sigma));
end