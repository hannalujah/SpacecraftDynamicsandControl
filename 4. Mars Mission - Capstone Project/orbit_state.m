function [r,r_dot] = orbit_state(r,Omega,i,theta,theta_dot)

% Since the motion is planar and circular, at any time we have
% r_dot = 0 i_r + r*theta_dot i_theta + 0 i_h

r_O = [r;0;0];
r_dot_O = [0;r*theta_dot;0];

ON = M3(theta)*M1(i)*M3(Omega);
NO = transpose(ON);

% Taking the O frame vectors to the N frame
r = (NO*r_O) / 1000;    
r_dot = (NO*r_dot_O) / 1000;

% r parameters are expressed in "km"
% r_dot parameters are expressed in "km/s"

end