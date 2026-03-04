function [r,r_dot] = orbit_state(r,Omega,i,theta,theta_dot)

% Since the motion is planar and circular, at any time we have
% r_dot = 0 i_r + r*theta_dot i_theta + 0 i_h

r_B = [r;0;0];
r_dot_B = [0;r*theta_dot;0];

NB = M3(theta)*M1(i)*M3(Omega);
BN = transpose(NB);
% Taking the B frame vectors to the N frame
r = BN*r_B;
r_dot = BN*r_dot_B;
end