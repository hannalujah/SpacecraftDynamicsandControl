function T = t_matrix(s,m)
    t1 = s;
    t2 = cross(s,m) / norm(cross(s,m));
    t3 = cross(t1,t2);
    
    T = [t1,t2,t3];   
end