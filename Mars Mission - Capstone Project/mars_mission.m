%% Project Constants
% Daughter-Craft
h_dc = 400*1000; % m
R_mars = 3396.19*1000; % m
r_LMO = R_mars + h_dc; % m

mio_mars = 42828.3*10^9; % m^3/s^2
theta_dot_LMO = sqrt(mio_mars / (r_LMO^3));  % rad/s

sigma_BN_0 = [0.3;-0.4;0.5];
w_BN_0 = [1.00;1.75;-2.20] * pi/180; % rad/s

I_dc = diag([10;5;7.5]); % kg.m^2

RA_dc = 20 * pi/180; % DC right ascention
i_dc = 30 * pi/180;  % DC Inclination
theta_dc_0 = 60 * pi/180; % DC initial angle

% Mother-Craft
T_mc = (1*24*60 + 37)*60; % sec
r_GMO = 20424.2*1000; % m
theta_dot_GMO = sqrt(mio_mars / (r_GMO^3));  % rad/s

RA_mc = 0;
i_mc = 0;
theta_mc_0 = 250 * pi/180; % MC initial angle


%% Module 2 - Orbits

% Task 1 - Orbit Simulation
fprintf("Task 1 - Orbit Simulation \n");

theta_dc_450 = rem(theta_dot_LMO*450 + theta_dc_0, 2*pi);
[r_dc_450, r_dot_dc_450] = orbit_state(r_LMO, RA_dc, i_dc, theta_dc_450, theta_dot_LMO);

fprintf("Daughter craft, t = 450s \n");
fprintf("r = [%.3f, %.3f, %.3f] km\n", r_dc_450(1)/1000, r_dc_450(2)/1000, r_dc_450(3)/1000);
fprintf("r_dot = [%.3f, %.3f, %.3f] km/s\n", r_dot_dc_450(1)/1000, r_dot_dc_450(2)/1000, r_dot_dc_450(3)/1000);

theta_mc_1150 = rem(theta_dot_GMO*1150 + theta_mc_0, 2*pi);
[r_mc_1150, r_dot_mc_1150] = orbit_state(r_GMO, RA_mc, i_mc, theta_mc_1150, theta_dot_GMO);

fprintf("Mother craft, t = 1150s \n");
fprintf("r = [%.3f, %.3f, %.3f] km\n", r_mc_1150(1)/1000, r_mc_1150(2)/1000, r_mc_1150(3)/1000);
fprintf("r_dot = [%.3f, %.3f, %.3f] km/s\n", r_dot_mc_1150(1)/1000, r_dot_mc_1150(2)/1000, r_dot_mc_1150(3)/1000);

% Task 2 - Orbit Frame Orientation
fprintf("Task 2 - Orbit Frame Orientation \n");
HN = orbit_frame_dcm(300);

%% Module 3 - Reference Frame Orientation

% Task 3 - Sun-pointing Reference Frame Orientation
fprintf("Task 3 - Sun-pointing Reference Frame Orientation \n");
RsN = RsN_DCM();
w_RsN = [0;0;0];

fprintf("Sun Pointing RFO\n");
fprintf("[RsN] = [%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f]\n", RsN(1,1), RsN(1,2), RsN(1,3), RsN(2,1), RsN(2,2), RsN(2,3), RsN(3,1), RsN(3,2), RsN(3,3));

% Task 4 - Nadir-pointing Reference Frame Orientation
fprintf("Task 4 - Nadir-pointing Reference Frame Orientation \n");
RnN = RnN_DCM(330);
w_RnN_H = [0;0;theta_dot_LMO];
w_RnN = transpose(HN)*w_RnN_H;

fprintf("Nadir Pointing RFO\n");
fprintf("[RnN] = [%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f]\n", RnN(1,1), RnN(1,2), RnN(1,3), RnN(2,1), RnN(2,2), RnN(2,3), RnN(3,1), RnN(3,2), RnN(3,3));
fprintf("W_RnN = [%.9f %.9f %.9f]\n", w_RnN(1), w_RnN(2), w_RnN(3));

% Task 5 - GMO-pointing Reference Frame Orientation
fprintf("Task 5 - GMO-pointing Reference Frame Orientation \n");

RcN = RcN_DCM(330);
fprintf("GMO Pointing RFO\n");
fprintf("[RcN] = [%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f]\n", RcN(1,1), RcN(1,2), RcN(1,3), RcN(2,1), RcN(2,2), RcN(2,3), RcN(3,1), RcN(3,2), RcN(3,3));

w_RcN = RcN_omega(330);
fprintf("W_RcN = [%.9f %.9f %.9f]\n", w_RcN(1), w_RcN(2), w_RcN(3));

%% Module 4 - Attitude Evaluation and Simulator

% Task 6 - Attitude Error Evaluation
fprintf("Task 6 - Attitude Error Evaluation \n");
t0 = 0;
% Sun pointing attitude errors
fprintf("Sun Pointing Attitude Error\n");
RsN_0 = RsN_DCM();
w_RsN = [0;0;0];

[sigma_BR_SP_0, w_BR_SP_0] = attitudeErrorEval(sigma_BN_0, w_BN_0, RsN_0, w_RsN);
fprintf("Sigma BR: [%.6f %.6f %.6f]\n", sigma_BR_SP_0(1), sigma_BR_SP_0(2), sigma_BR_SP_0(3));
fprintf("Omega BR: [%.6f %.6f %.6f]\n\n", w_BR_SP_0(1), w_BR_SP_0(2), w_BR_SP_0(3));

% Nadir Pointing Attitude Errors
fprintf("Nadir Pointing Attitude Error\n");
RnN_0 = RnN_DCM(t0);
w_RnN_0 = w_RnN;

[sigma_BR_NP_0, w_BR_NP_0] = attitudeErrorEval(sigma_BN_0, w_BN_0, RnN_0, w_RnN_0);
fprintf("Sigma BR: [%.6f %.6f %.6f]\n", sigma_BR_NP_0(1), sigma_BR_NP_0(2), sigma_BR_NP_0(3));
fprintf("Omega BR: [%.6f %.6f %.6f]\n\n", w_BR_NP_0(1), w_BR_NP_0(2), w_BR_NP_0(3));

% GMO Pointing Attitude Errors
fprintf("GMO Pointing Attitude Error\n");
RcN_0 = RcN_DCM(t0);
w_RcN_0 = RcN_omega(t0);

[sigma_BR_GP_0, w_BR_GP_0] = attitudeErrorEval(sigma_BN_0, w_BN_0, RcN_0, w_RcN_0);
fprintf("Sigma BR: [%.6f %.6f %.6f]\n", sigma_BR_GP_0(1), sigma_BR_GP_0(2), sigma_BR_GP_0(3));
fprintf("Omega BR: [%.6f %.6f %.6f]\n\n", w_BR_GP_0(1), w_BR_GP_0(2), w_BR_GP_0(3));


% Task 7 - Numerical Attitude Simulator
fprintf("Task 7 - Numerical Attitude Simulator \n");
sigma_BN = sigma_BN_0;
w_BN = w_BN_0;

t0 = 0;
tf = 600;
dt = 0.001;

t = t0:dt:tf;
u1 = [0;0;0];
u2 = [0.01;-0.01;0.02];

for i = t
   if (i == 500)
      H = I_dc*w_BN;
      T = 0.5*transpose(w_BN)*I_dc*w_BN;
      
      BN = sigmaToDCM(sigma_BN);
      H_N = transpose(BN)*H;
      
      fprintf("t = 500s\n");
      fprintf("H_B = [%.6f %.6f %.6f]\n", H(1), H(2), H(3));
      fprintf("H_N = [%.6f %.6f %.6f]\n", H_N(1), H_N(2), H_N(3));
      fprintf("T = %.9f J\n", T);
      fprintf("Sigma_BN = [%.6f %.6f %.6f]\n\n", sigma_BN(1),sigma_BN(2),sigma_BN(3));
   end
    
   sigma_dot_BN = (1/4)*B_sigma(sigma_BN)*w_BN;
   w_dot_BN = I_dc\(-tilde(w_BN)*I_dc*w_BN + u1);
   
   sigma_BN = sigma_dot_BN*dt + sigma_BN;
   if (norm(sigma_BN) > 1)
       sigma_BN = sigmaToSigmaS(sigma_BN);
   end
   w_BN = w_dot_BN*dt + w_BN;
end

sigma_BN = sigma_BN_0;
w_BN = w_BN_0;

for i = t
   if (i == 100)      
      fprintf("t = 100s, non-zero u applied\n");
      fprintf("Sigma_BN = [%.6f %.6f %.6f]\n\n", sigma_BN(1),sigma_BN(2),sigma_BN(3));
   end
    
   sigma_dot_BN = (1/4)*B_sigma(sigma_BN)*w_BN;
   w_dot_BN = I_dc\(-tilde(w_BN)*I_dc*w_BN + u2);
   
   sigma_BN = sigma_dot_BN*dt + sigma_BN;
   if (norm(sigma_BN) > 1)
       sigma_BN = sigmaToSigmaS(sigma_BN);
   end
   w_BN = w_dot_BN*dt + w_BN;  
end

%% Module 5 - Complete the Mission

I_max = 10;
I_min = 5;
T = 120;

K = 4*I_max^2 / ((T^2) * I_min);
P = 2*I_max / T;

fprintf("K = %.9f, P = %.9f\n", K, P);

dt = 0.1;
tf = 400;

idx15 = round(15/dt);
idx100 = round(100/dt);
idx200 = round(200/dt);
idx400 = round(400/dt);

% Task 8 - Sun Pointing Control
fprintf("Task 8 - Sun Pointing Control \n");
[t8, sigma8, w8, u8] = simulate_mode("sun",0, tf, dt, sigma_BN_0, w_BN_0, K, P, I_dc);
fprintf("Sigma_BN at t = 15s: [%.6f %.6f %.6f]\n", sigma8(1,idx15), sigma8(2,idx15), sigma8(3,idx15));
fprintf("Sigma_BN at t = 100s: [%.6f %.6f %.6f]\n", sigma8(1,idx100), sigma8(2,idx100), sigma8(3,idx100));
fprintf("Sigma_BN at t = 200s: [%.6f %.6f %.6f]\n", sigma8(1,idx200), sigma8(2,idx200), sigma8(3,idx200));
fprintf("Sigma_BN at t = 400s: [%.6f %.6f %.6f]\n", sigma8(1,idx400), sigma8(2,idx400), sigma8(3,idx400));

% Task 9 - Nadir Pointing Control
fprintf("Task 9 - Nadir Pointing Control \n");
[t9, sigma9, w9, u9] = simulate_mode("nadir",0, tf, dt, sigma_BN_0, w_BN_0, K, P, I_dc);
fprintf("Sigma_BN at t = 15s: [%.6f %.6f %.6f]\n", sigma9(1,idx15), sigma9(2,idx15), sigma9(3,idx15));
fprintf("Sigma_BN at t = 100s: [%.6f %.6f %.6f]\n", sigma9(1,idx100), sigma9(2,idx100), sigma9(3,idx100));
fprintf("Sigma_BN at t = 200s: [%.6f %.6f %.6f]\n", sigma9(1,idx200), sigma9(2,idx200), sigma9(3,idx200));
fprintf("Sigma_BN at t = 400s: [%.6f %.6f %.6f]\n", sigma9(1,idx400), sigma9(2,idx400), sigma9(3,idx400));

% Task 10 - GMO Pointing Control
fprintf("Task 10 - GMO Pointing Control \n");
[t10, sigma10, w10, u10] = simulate_mode("gmo",0, tf, dt, sigma_BN_0, w_BN_0, K, P, I_dc);
fprintf("Sigma_BN at t = 15s: [%.6f %.6f %.6f]\n", sigma10(1,idx15), sigma10(2,idx15), sigma10(3,idx15));
fprintf("Sigma_BN at t = 100s: [%.6f %.6f %.6f]\n", sigma10(1,idx100), sigma10(2,idx100), sigma10(3,idx100));
fprintf("Sigma_BN at t = 200s: [%.6f %.6f %.6f]\n", sigma10(1,idx200), sigma10(2,idx200), sigma10(3,idx200));
fprintf("Sigma_BN at t = 400s: [%.6f %.6f %.6f]\n", sigma10(1,idx400), sigma10(2,idx400), sigma10(3,idx400));

idx300 = round(300/dt);
idx2100 = round(2100/dt);
idx3400 = round(3400/dt);
idx4400 = round(4400/dt);
idx5600 = round(5600/dt);

tf = 6500;
% Task 11 - Mission Scenario Simulation
fprintf("Task 11 - Mission Scenario Simulation \n");
[t11, sigma11, w11, u11] = final_simulation(0, tf, dt, sigma_BN_0, w_BN_0, K, P, I_dc);
fprintf("Sigma_BN at t = 300s: [%.6f %.6f %.6f]\n", sigma11(1,idx300), sigma11(2,idx300), sigma11(3,idx300));
fprintf("Sigma_BN at t = 2100s: [%.6f %.6f %.6f]\n", sigma11(1,idx2100), sigma11(2,idx2100), sigma11(3,idx2100));
fprintf("Sigma_BN at t = 3400s: [%.6f %.6f %.6f]\n", sigma11(1,idx3400), sigma11(2,idx3400), sigma11(3,idx3400));
fprintf("Sigma_BN at t = 4400s: [%.6f %.6f %.6f]\n", sigma11(1,idx4400), sigma11(2,idx4400), sigma11(3,idx4400));
fprintf("Sigma_BN at t = 5600s: [%.6f %.6f %.6f]\n", sigma11(1,idx5600), sigma11(2,idx5600), sigma11(3,idx5600));
