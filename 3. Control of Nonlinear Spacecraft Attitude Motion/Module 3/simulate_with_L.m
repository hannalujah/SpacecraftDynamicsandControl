function sigmaNormT = simulate_with_L(isKnownL, Tquery, L)

sigma_BN = [0.1;0.2;-0.1];
w_BN     = [30;10;-20]*pi/180;

I = diag([100;75;80]);
K = 5;
P = 10*eye(3);
f = 0.05;

dt = 0.1;
tf = 100;
t  = 0:dt:tf;

kQuery = round(Tquery/dt) + 1;
k = 0;
sigmaNormT = NaN;

for ti = t
    k = k + 1;

    % Reference sigma and sigma_dot
    sigma_RN = [0.2*sin(f*ti); 0.3*cos(f*ti); -0.3*sin(f*ti)];
    sigma_dot_RN = f*[0.2*cos(f*ti); -0.3*sin(f*ti); -0.3*cos(f*ti)];

    % omega_RN in R frame
    w_RN_R = 4 * (B(sigma_RN) \ sigma_dot_RN);

    % Attitude error sigma_BR
    sigma_BR = mrpAddition(sigma_BN, -sigma_RN);
    if norm(sigma_BR) > 1, sigma_BR = sShadow(sigma_BR); end
    
    % C_BR maps R->B
    C_BR = mrp2dcm(sigma_BR);

    % reference omega in B
    w_RN_B = C_BR * w_RN_R;

    % rate error
    w_BR = w_BN - w_RN_B;

    if k == 1
        wdot_RN_B = zeros(3,1);   % bootstrap
    else
        wdot_RN_B = (w_RN_B - w_RN_B_prev)/dt;
    end
    w_RN_B_prev = w_RN_B;

    % Controller core
    u = -K*sigma_BR - P*w_BR ...
        + I*(wdot_RN_B - cross(w_BN, w_RN_B)) ...
        + cross(w_BN, I*w_BN);

    % Q7 only: subtract known L
    if isKnownL
        u = u - L;
    end

    % Record answer
    if k == kQuery
        sigmaNormT = norm(sigma_BR);
    end

    % Plant propagation (L always acts)
    sigma_dot_BN = 0.25 * B(sigma_BN) * w_BN;
    w_dot_BN     = I \ (-cross(w_BN, I*w_BN) + u + L);

    sigma_BN = sigma_BN + dt*sigma_dot_BN;
    w_BN     = w_BN     + dt*w_dot_BN;

    if norm(sigma_BN) > 1, sigma_BN = sShadow(sigma_BN); end
end
end

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
