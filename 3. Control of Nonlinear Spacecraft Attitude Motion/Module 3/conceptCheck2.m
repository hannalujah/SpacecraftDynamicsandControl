% Question 5,6,7
sigma_BN_0 = [0.1;0.2;-0.1];
w_BN_0 = [30;10;-20] * pi/180;
f = 0.05;

tf = 100;
dt = 0.1;

t = 0:dt:tf;

sigma_BN_i = sigma_BN_0;
w_BN_i = w_BN_0;
sigma_BR_i = 0;
K = 5;
P = 10*eye(3);
L = 0;
I = diag([100;75;80]);

k20 = round(20/dt) + 1;
k80 = round(80/dt) + 1;
k70 = round(70/dt) + 1;
k = 0;
w_RN_prev = [];     % in R-frame

sigmaNorm20_q5 = 0;
sigmaNorm80_q6 = 0;
sigmaNorm70_q7 = 0;

L6_7 = [0.5;-0.3;0.2];

for ti = t
  k = k + 1;

  % Reference sigma and sigma_dot
  sigma_RN_i = [0.2*sin(f*ti); 0.3*cos(f*ti); -0.3*sin(f*ti)];
  sigma_dot_RN_i = f*[0.2*cos(f*ti); -0.3*sin(f*ti); -0.3*cos(f*ti)];

  sigma_RN_i_1 = [0.2*sin(f*(ti+dt)); 0.3*cos(f*(ti+dt)); -0.3*sin(f*(ti+dt))];
  sigma_dot_RN_i_1 = f*[0.2*cos(f*(ti+dt)); -0.3*sin(f*(ti+dt)); -0.3*cos(f*(ti+dt))];

  % omega_R/N in R frame
  w_RN_i   = 4 * (B(sigma_RN_i)   \ sigma_dot_RN_i);
  w_RN_i_1 = 4 * (B(sigma_RN_i_1) \ sigma_dot_RN_i_1);
  w_dot_RN_i = (w_RN_i_1 - w_RN_i) / dt;   % in R frame

  % Attitude error sigma_B/R
  sigma_BR_i = mrpAddition(sigma_BN_i, -sigma_RN_i);

  % Shadow set on sigma_BR (recommended)
  if norm(sigma_BR_i) > 1
      sigma_BR_i = sShadow(sigma_BR_i);
  end

  % DCM C_B/R mapping R->B
  C_BR = mrp2dcm(sigma_BR_i);

  % Rotate reference rates into B frame
  w_RN_B     = C_BR * w_RN_i;
  
  % Rate error in B
  delta_w_i = w_BN_i - w_RN_B;
  
  w_dot_RN_B = C_BR * w_dot_RN_i + cross(delta_w_i, w_RN_B);

  % Control (all in B frame now)
  u5 = -P*delta_w_i - K*sigma_BR_i;
 
  % Save norm at 40 s
  if k == k20
    sigmaNorm20_q5 = norm(sigma_BR_i);
  end

  % Plant propagation
  sigma_dot_BN_i = 0.25 * B(sigma_BN_i) * w_BN_i;
  w_dot_BN_i = I \ (-cross(w_BN_i, I*w_BN_i) + u5 + L);

  sigma_BN_i = sigma_BN_i + sigma_dot_BN_i * dt;
  w_BN_i     = w_BN_i     + w_dot_BN_i     * dt;

  % Shadow set on sigma_BN
  if norm(sigma_BN_i) > 1
      sigma_BN_i = sShadow(sigma_BN_i);
  end
end

fprintf('Norm of sigma at t = 20s is : %.6f\n', sigmaNorm20_q5);

L = [0.5; -0.3; 0.2];

sigmaNorm80_q6 = simulate_with_L(false, 80, L); % Q6
sigmaNorm70_q7 = simulate_with_L(true,  70, L); % Q7

fprintf('Q6: ||sigma_BR(80)|| = %.6f\n', sigmaNorm80_q6);
fprintf('Q7: ||sigma_BR(70)|| = %.6f\n', sigmaNorm70_q7);

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