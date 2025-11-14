function dQ = diffQ(q)

    dQ = 0.5* (eye(3) + tilda(q) + q*transpose(q));

end