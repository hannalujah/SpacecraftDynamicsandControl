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
w_RsN = 0;
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
% The 

% Task 5 - GMO-pointing Reference Frame Orientation
fprintf("Task 5 - GMO-pointing Reference Frame Orientation \n");

RcN = RcN_DCM(330);
fprintf("GMO Pointing RFO\n");
fprintf("[RcN] = [%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f]\n", RcN(1,1), RcN(1,2), RcN(1,3), RcN(2,1), RcN(2,2), RcN(2,3), RcN(3,1), RcN(3,2), RcN(3,3));

w_RcN = RcN_omega(330);
fprintf("W_RcN = [%.9f %.9f %.9f]\n", w_RcN(1), w_RcN(2), w_RcN(3));

% Needs serious revision on the w_RcN - the answer is not correct
