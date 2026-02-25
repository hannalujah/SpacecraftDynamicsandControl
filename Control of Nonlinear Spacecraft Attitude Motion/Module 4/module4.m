%% Concept check 2

K = 0.11;              % /s^2
P = 3*eye(3);          % m
I = diag([100;75;80]); % kg.m^2
 
sigma_BN_0 = [0.1;0.2;-0.1];
w_BN_0 = [3;2;-1]*pi/180;     % rad/s


dt = 0.1;
tf = 120;
t = 0:dt:tf;

sigma_BN_i = sigma_BN_0;
w_BN_i = w_BN_0;

sigma_BN = sigma_BN_0;
w_BN = w_BN_0;
for i = t
   %u = ;
   w_dot_BN_i = -P*w_BN_i - ...
       (w_BN_i*transpose(w_BN_i) + ( (4*K/(1+norm(sigma_BN_i)^2)) - (norm(w_BN_i)/2) )*eye(3))*sigma_BN_i;
   sigma_dot_BN_i = (1/4)*B_sigma(sigma_BN_i)*w_BN_i; 
   
   w_BN_i = w_BN_i + w_dot_BN_i*dt;
   sigma_BN_i = sigma_BN_i + sigma_dot_BN_i*dt;
   
   if (norm(sigma_BN_i) > 1)
      sigma_BN_i = sigmaToSigmaS(sigma_BN_i); 
   end
       
    sigma_BN = [sigma_BN, sigma_BN_i];
    w_BN = [w_BN, w_BN_i];
end

figure
plot(t, sigma_BN(1,1:1201), 'LineWidth', 1.5)
hold on
plot(t, sigma_BN(2,1:1201), 'LineWidth', 1.5)
plot(t, sigma_BN(3,1:1201), 'LineWidth', 1.5)

legend('\sigma_1','\sigma_2','\sigma_3','Location','best')
xlabel('Time (s)')
ylabel('MRP')
title('MRP Time History')
grid on

