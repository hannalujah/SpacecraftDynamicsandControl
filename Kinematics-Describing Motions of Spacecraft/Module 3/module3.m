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
[e,phi] = prvCalc(FN);

%% Concept check 5/6 - Euler Parameter Relationship to DCM

% Question 1
beta = [0.235702, 0.471405, -0.471405, 0.707107];
DCM = betaToDCM(beta);

% Question 3
BN = [-0.529403, -0.467056, 0.708231;
    -0.474115, -0.529403, -0.703525;
    0.703525, -0.708231, 0.0588291];

beta = DCMToBeta(BN);

%Question 4
theta1 = 20*pi/180;
theta2 = 10*pi/180;
theta3 = -10*pi/180;
euler321 = M1(theta3)*M2(theta2)*M3(theta1);

beta = DCMToBeta(euler321);

%% Concept Check 7 - Euler Parameter Addition

%Question 1
beta_BN = [0.774597, 0.258199, 0.516398, 0.258199];
beta_FB = [0.359211, 0.898027, 0.179605, 0.179605];

beta_FN = betaAddition(beta_BN, beta_FN, 0)

% Question 2
beta_FN = [0.359211, 0.898027, 0.179605, 0.179605];
beta_BN = [-0.3777964, 0.755929, 0.377964, 0.377964];

BN = betaToDCM(beta_BN);
beta_NB = DCMToBeta(transpose(BN));
beta_FB = betaAddition(beta_NB, beta_FN, 0)


FB = FN * transpose(BN);
beta_FB = DCMToBeta(FB);

FB_prime = betaToDCM(beta_FB);

