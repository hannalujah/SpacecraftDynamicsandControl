function matrix = diffEA(theta,phi)
% for an euler321 sequence
    matrix = (1/cos(theta)) * [0, sin(phi), cos(phi);
        0, cos(phi)*cos(theta), -sin(phi)*cos(theta);
        cos(theta), sin(phi)*sin(theta), cos(phi)*sin(theta)];
end