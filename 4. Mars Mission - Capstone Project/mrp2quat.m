function q = mrp2quat(sigma)
    s2 = dot(sigma,sigma);
    q0 = (1 - s2)/(1 + s2); % scalar
    qv = 2*sigma/(1 + s2);  % vector
    q = [q0 qv(:)'];        % row vector [q0 q1 q2 q3]
end