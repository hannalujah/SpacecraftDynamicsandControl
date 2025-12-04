function DCM = qToDCM(q)
q = transpose(q);
        q_tilda = [0, -q(3), q(2);
        q(3), 0, -q(1);
        -q(2), q(1), 0];
    qNorm = transpose(q)*q;
      
    DCM = (1/(1+qNorm))*((1-qNorm)*eye(3) + 2*q*transpose(q) - 2*q_tilda);

end