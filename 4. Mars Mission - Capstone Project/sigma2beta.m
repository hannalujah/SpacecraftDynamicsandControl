function betaHist = sigma2beta(sigmaHist)

[~,col] = size(sigmaHist);
betaHist = zeros(4,col);

for i = 1:col
    sigma = sigmaHist(:,i);
    ss = norm(sigma);
    b0 = (1 - ss^2)/(1 + ss^2);
    bv = (2/(1+ss^2))*sigma;

    betaHist(:,i) = [b0;bv];
end

end