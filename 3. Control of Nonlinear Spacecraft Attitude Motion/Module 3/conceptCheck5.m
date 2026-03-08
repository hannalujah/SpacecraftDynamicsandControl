I = diag([100;75;80]); % Intertia in kg.m^2

sigma_BN_0 = [0.1;0.2;-0.1]; % Initial MRP
omega_BN_0 = [30;10;-20] * pi/180; % Initial Angular Velocity

K = 5; % N.m
P = diag([22.361; 19.365; 20]);
L = 0; % External torque

% With the reference frame being the same as the inertial frame we have
% [I]w_dot = -K.sigma - [P]w
dt = 0.1;
t = 0:dt:120;
sigma_i = sigma_BN_0;
omega_i = omega_BN_0;

normSigma30 = 0;

for i = t
    sigma_dot = (1/4)*B_sigma(sigma_i)*omega_i;
    omega_dot = I\(-K*sigma_i - P*omega_i);
    
    sigma_i = sigma_i + sigma_dot*dt;
    if (norm(sigma_i) > 1)
       sigma_i = sigmaToSigmaS(sigma_i); 
    end
    
    omega_i = omega_i + omega_dot*dt;
    
    if(i == 30) 
        normSigma30 = norm(sigma_i);
    end
end

fprintf('||sigma_B/R|| = %.6f\n', normSigma30);
