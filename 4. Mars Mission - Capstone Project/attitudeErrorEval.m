function [sigma_BR, w_BR] = attitudeErrorEval(sigma_BN, w_BN, RN, w_RN)

BN = sigmaToDCM(sigma_BN);
BR = BN*transpose(RN);

sigma_BR = DCMToSigma(BR);
w_BR = w_BN - BN*w_RN;

end