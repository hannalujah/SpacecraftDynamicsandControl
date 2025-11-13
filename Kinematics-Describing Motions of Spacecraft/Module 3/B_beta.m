function B = B_beta(beta)

    B = [beta(1), -beta(2), -beta(3), -beta(4);
        beta(2), beta(1), -beta(4), beta(3);
        beta(3), beta(4), beta(1), -beta(2);
        beta(4), -beta(3), beta(2), beta(1)];


end