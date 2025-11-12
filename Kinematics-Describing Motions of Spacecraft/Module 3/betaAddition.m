function beta = betaAddition(betaP, betaZ, M)

    if M == 0
       C = [betaZ(1), -betaZ(2), -betaZ(3), -betaZ(4);
           betaZ(2), betaZ(1), betaZ(4), -betaZ(3);
           betaZ(3), -betaZ(4), betaZ(1), betaZ(2);
           betaZ(4), betaZ(3), -betaZ(2), betaZ(1)];
       
       beta = C*transpose(betaP);
    else
        C = [betaP(1), -betaP(2), -betaP(3), -betaP(4);
           betaP(2), betaP(1), -betaP(4), betaP(3);
           betaP(3), betaP(4), betaP(1), -betaP(2);
           betaP(4), betaP(3), betaP(2), betaP(1)];
       beta = C*transpose(betaZ);      
        
    end

end