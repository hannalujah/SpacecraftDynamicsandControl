function [e,phi] = prvCalc(C)

    phi = acos((C(1,1) + C(2,2) + C(3,3) - 1)/2);
    
    e = [C(2,3) - C(3,2);
        C(3,1) - C(1,3);
        C(1,2) - C(2,1)] / (2*sin(phi));
    
end