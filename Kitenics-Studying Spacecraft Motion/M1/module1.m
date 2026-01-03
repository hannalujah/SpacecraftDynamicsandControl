%% Concept Check 2 - Kinetic Energy
m1 = 1; m2 = 1; 
m3 = 2; m4 = 2;
M = m1 + m2 + m3 + m4;

R1 = [1;-1;2]; R2 = [-1;-3;2];
R3 = [2;-1;-1]; R4 = [3;-1;-2];

R1_dot = [2;1;1]; R2_dot = [0;-1;1];
R3_dot = [3;2;-1]; R4_dot = [0;0;1];

Rc = (1/M)*(m1*R1 + m2*R2 + m3*R3 + m4*R4);
Rc_dot = (1/M)*(m1*R1_dot + m2*R2_dot + m3*R3_dot + m4*R4_dot);

r1 = R1 - Rc; r2 = R2 - Rc; 
r3 = R3 - Rc; r4 = R4 - Rc;
r1_dot = R1_dot - Rc_dot; r2_dot = R2_dot - Rc_dot; 
r3_dot = R3_dot - Rc_dot; r4_dot = R4_dot - Rc_dot;

transEnergy = 0.5*M*dot(Rc_dot,Rc_dot);
rotEnergy = 0.5*(m1*dot(r1_dot,r1_dot) + m2*dot(r2_dot,r2_dot) + m3*dot(r3_dot,r3_dot) + m4*dot(r4_dot,r4_dot));


%% Concept Check 4 - Angular Momentum
m1 = 1; m2 = 1; 
m3 = 2; m4 = 2;

R1 = [1;-1;2]; R2 = [-1;-3;2];
R3 = [2;-1;-1]; R4 = [3;-1;-2];

R1_dot = [2;1;1]; R2_dot = [0;-1;1];
R3_dot = [3;2;-1]; R4_dot = [0;0;1];

M = m1 + m2 + m3 + m4;

Rc = (1/M)*(m1*R1 + m2*R2 + m3*R3 + m4*R4);
Rc_dot = (1/M)*(m1*R1_dot + m2*R2_dot + m3*R3_dot + m4*R4_dot);

% Case 1
Rp = [0;0;0]; % Origin
Rp_dot = [0;0;0];

% Case 2
Rp = Rc;
Rp_dot = Rc_dot;

sigma1 = R1 - Rp; sigma2 = R2 - Rp;
sigma3 = R3 - Rp; sigma4 = R4 - Rp;

sigma1_dot = R1_dot - Rp_dot; sigma2_dot = R2_dot - Rp_dot;
sigma3_dot = R3_dot - Rp_dot; sigma4_dot = R4_dot - Rp_dot;

H = m1*cross(sigma1,sigma1_dot) + m2*cross(sigma2,sigma2_dot) + m3*cross(sigma3,sigma3_dot) + m4*cross(sigma4,sigma4_dot);

%% Concept Check 5 - Rigid Body Angular Momentum
theta1 = -10*pi/180; % 3
theta2 = 10*pi/180; %2 
theta3 = 5*pi/180; %1

euler321 = M1(theta3)*M2(theta2)*M3(theta1);
omega_N_Frame = [0.01;-0.01;0.01];
omega_B_Frame = euler321*omega_N_Frame;

Inertia_B_Frame = [10 1 -1; 1 5 1; -1 1 8];

H_B_Frame = Inertia_B_Frame*omega_B_Frame;


%% Concept Check 6 - Parallel Axis Theorem
theta1 = -10*pi/180; % 3
theta2 = 10*pi/180; %2 
theta3 = 5*pi/180; %1

euler321 = M1(theta3)*M2(theta2)*M3(theta1);
BN = euler321; 
M = 12.5;

Ic_B_Frame = [10 1 -1; 1 5 1; -1 1 8];
Rc_P_N_Frame = [-0.5;0.5;0.25];

Rc_P_B_Frame = BN*Rc_P_N_Frame;

Ip_B_Frame = Ic_B_Frame + M*tilda(Rc_P_B_Frame)*transpose(tilda(Rc_P_B_Frame));

%% Concept Check 6.1 
sigma_DB = [0.1;0.2;0.3];
DB = sigmaToDCM(sigma_DB);

Ic_B = [10 1 -1; 1 5 1; -1 1 8];

Ic_D = DB*Ic_B*transpose(DB);

[V,D] = eig(Ic_B);


%% Concept Check 7 - Kinetic Energy
Ic_B_Frame = [10 1 -1; 1 5 1; -1 1 8];
omega_B_Frame = [0.01;-0.01;0.01];

T_rot = 0.5*transpose(omega_B_Frame)*Ic_B_Frame*omega_B_Frame;

%% Concept Check 8




