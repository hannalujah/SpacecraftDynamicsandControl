%% Concept check 2 - Pricipal Rotation Parameter Relation to DCM

C = [0.925417, 0.336824, 0.173648;
    0.0296956, -0.521281, 0.852869;
    0.377786, -0.784102, -0.492404];

[e,phi] = prvCalc(C);

%% Concept check 3 - Principal Rotation Addition
% Q2
theta1 = 20*pi/180;
theta2 = -10*pi/180;
theta3 = 120*pi/180;

euler321 = M1(theta3)*M2(theta2)*M3(theta1);
[e,phi] = prvCalc(euler321);

% Q3
FB = [1,0,0;
    0,0,1;
    0,-1,0];
BN = FB;
FN = FB*BN;
[e,phi] = prvCalc(FN)

