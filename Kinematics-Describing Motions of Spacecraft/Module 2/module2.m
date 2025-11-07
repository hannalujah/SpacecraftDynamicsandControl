%% Eigenvalue and eigenvector

A = [-0.87097 0.45161 0.19355;
    -0.19355 -0.67742 0.70968;
    0.45161 0.58065 0.67742];

[V,D] = eig(A);

 %% Addition / Subtraction
BN = [1/3, 2/3, -2/3;
    0, sqrt(2)/2, sqrt(2)/2;
    2*sqrt(2)/3, -sqrt(2)/6, sqrt(2)/6];

FN = [3/4, -1/2, sqrt(3)/4;
    -1/2, 0, sqrt(3)/2;
    -sqrt(3)/4, -2*sqrt(3)/4, -1/4];

BF = BN * transpose(FN);
 

%% DCM rates
BN = A;
w_tilda = [0, -0.3, 0.2;
    0.3, 0, -0.1;
    -0.2, 0.1, 0];
dBN = -1*w_tilda*BN;

%% Euler Angles

dToR = pi/180;
theta1 = 10*dToR;
theta2 = 20*dToR;
theta3 = 30*dToR;

euler321 = M1(theta3)*M2(theta2)*M3(theta1);
C = euler321;

theta1_euler313 = atan2(C(3,1), -C(3,2)) / dToR;
theta2_euler313 = acos(C(3,3)) / dToR;
theta3_euler313 = atan2(C(1,3), C(2,3)) / dToR;

% --- 
% Concept Check 8
% ---

BN_theta1 = 10*dToR;
BN_theta2 = 20*dToR;
BN_theta3 = 30*dToR;

BN_euler321 = M1(BN_theta3)*M2(BN_theta2)*M3(BN_theta1);

RN_theta1 = -5*dToR;
RN_theta2 = 5*dToR;
RN_theta3 = 5*dToR;

RN_euler321 = M1(RN_theta3)*M2(RN_theta2)*M3(RN_theta1);

BR_euler321 = BN_euler321*transpose(RN_euler321);

C = BR_euler321;
BR_theta1 = atan2(C(1,2),C(1,1)) / dToR;
BR_theta2 = -asin(C(1,3)) / dToR;
BR_theta3 = atan2(C(2,3), C(3,3)) / dToR;

%% Concept Check 9
% Integration

t = 1:1:60;
psi0 = 40*pi/180;
theta0 = 30*pi/180;
phi0 = 80*pi/180;

EA0 = [psi0; theta0; phi0];
EAi = EA0;
norm = 0;

for i = t
    wi = [sin(0.1*i); 0.01; cos(0.1*i)] * 20 * pi/180;
    EAiDiff = diffEA(EAi(2), EAi(3)) * wi;
    EAi = EAiDiff + EAi;
    
    for j = 1:3
       if EAi(j) > 2*pi
           EAi(j) = EAi(j) - 2*pi;
       elseif (EAi(j) < -2*pi)
               EAi(j) = EAi(j) + 2*pi;
       end
    end
    
    if (i == 42)
        norm = sqrt((EAi(1)^2) + (EAi(2)^2) + (EAi(3)^2));
        break;
    end
end
