function DCM = sigmaToDCM(sigma)

    norm_s = norm(sigma);
    C = (1 / ((1+norm_s^2)^2) ) * ((8 * tilda(sigma) * tilda(sigma)) - (4*(1-norm_s^2)*tilda(sigma)));
    C = C + eye(3);
    DCM = C;

end