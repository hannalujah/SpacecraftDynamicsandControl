sigma_BN_0 = [0.1;0.2;-0.1];
w_BN_0 = [3;1;-2] * pi/180;     
f = 0.05;

tf = 240;                         % 4 minutes
dt = 0.01;
t = 0:dt:tf;

K = 5;
P = 10*eye(3);
Ki = 0.005;
%Ki = 0;
I = diag([100;75;80]);

delta_L = [0.5;-0.3;0.2];         % unknown disturbance acting on plant
L = [0;0;0];                      % model torque (given as zero)

k45 = round(45/dt) + 1;
k35 = round(35/dt) + 1;

sigma_BN_i = sigma_BN_0;
w_BN_i     = w_BN_0;

sigmaNorm = NaN;

% Integral-control memory states
int_sigma = zeros(3,1);           % integral of sigma_BR dt
delta_w0  = [];                   % storing initial omega_BR

% For omega_RN derivative in B
w_RN_B_prev = [];

k = 0;
for ti = t
    k = k + 1;

    % Reference sigma and sigma_dot
    sigma_RN_i = [0.2*sin(f*ti); 0.3*cos(f*ti); -0.3*sin(f*ti)];
    sigma_dot_RN_i = f*[0.2*cos(f*ti); -0.3*sin(f*ti); -0.3*cos(f*ti)];

    % omega_R/N in R frame
    w_RN_R = 4 * (B(sigma_RN_i) \ sigma_dot_RN_i);

    % Attitude error sigma_B/R
    sigma_BR = mrpAddition(sigma_BN_i, -sigma_RN_i);
    if norm(sigma_BR) > 1
        sigma_BR = sShadow(sigma_BR);
    end

    % DCM C_B/R mapping R -> B
    C_BR = mrp2dcm(sigma_BR);

    % omega_R/N expressed in B
    w_RN_B = C_BR * w_RN_R;

    % omega_B/R in B
    delta_w = w_BN_i - w_RN_B;

    % Store delta_w0 at the start
    if isempty(delta_w0)
        delta_w0 = delta_w; 
    end

    % Approx w_dot_RN in B by finite difference in B
    if isempty(w_RN_B_prev)
        w_dot_RN_B = zeros(3,1);
    else
        w_dot_RN_B = (w_RN_B - w_RN_B_prev)/dt;
    end
    w_RN_B_prev = w_RN_B;

    % Integral term z
    int_sigma = int_sigma + sigma_BR*dt;        % Euler integral of sigma_BR
    z = K*int_sigma + I*(delta_w - delta_w0);

    % Nonlinear integral controller
    u = -K*sigma_BR - P*delta_w ...
        + I*(w_dot_RN_B - cross(w_BN_i, w_RN_B)) ...
        + cross(w_BN_i, I*w_BN_i) ...
        - P*(Ki*z);

    % recording answer at 45 s
    if k == k45
        sigmaNorm = norm(sigma_BR);
    end

    % Plant loop update
    sigma_dot_BN = 0.25 * B(sigma_BN_i) * w_BN_i;
    w_dot_BN     = I \ (-cross(w_BN_i, I*w_BN_i) + u + delta_L);
       
    sigma_BN_i = sigma_BN_i + sigma_dot_BN*dt;
    w_BN_i     = w_BN_i     + w_dot_BN*dt;

    if norm(sigma_BN_i) > 1
        sigma_BN_i = sShadow(sigma_BN_i);
    end
end

fprintf('||sigma_B/R|| = %.6f\n', sigmaNorm);

function s12 = mrpAddition(s1, s2)
  s1_2 = dot(s1,s1);
  s2_2 = dot(s2,s2);
  num = (1 - s2_2)*s1 + (1 - s1_2)*s2 - 2*cross(s1, s2);
  den = 1 + s1_2*s2_2 - 2*dot(s1, s2);
  s12 = num / den;
end

function B_sigma = B(sigma)
  B_sigma = (1 - norm(sigma)^2) * eye(3) + 2*tilda(sigma) + 2*sigma*(sigma.');
end

function shadow = sShadow(sigma)
  shadow = -sigma / (norm(sigma)^2);
end

function C = mrp2dcm(s)
  s2 = dot(s,s);
  S  = tilda(s);
  C  = eye(3) + (8*S*S - 4*(1 - s2)*S) / (1 + s2)^2;
end