function rotation = RcN_DCM(t)

h_dc = 400*1000;                                    % m
R_mars = 3396.19*1000;                              % m
r_LMO = R_mars + h_dc;                              % m
mio_mars = 42828.3*10^9;                            % m^3/s^2
theta_dot_LMO = sqrt(mio_mars / (r_LMO^3));         % rad/s
RA_dc = 20 * pi/180;                                % DC right ascention
i_dc = 30 * pi/180;                                 % DC Inclination
theta_dc_0 = 60 * pi/180;                           % DC initial angle
r_GMO = 20424.2*1000;                               % m
theta_dot_GMO = sqrt(mio_mars / (r_GMO^3));         % rad/s

RA_mc = 0;
i_mc = 0;
theta_mc_0  = 250 * pi/180;                         % MC initial angle

theta_dc_t  = rem(theta_dot_LMO*t + theta_dc_0, 2*pi);
[r_dc_t, ~] = orbit_state(r_LMO, RA_dc, i_dc, theta_dc_t, theta_dot_LMO);

theta_mc_t  = rem(theta_dot_GMO*t + theta_mc_0, 2*pi);
[r_mc_t, ~] = orbit_state(r_GMO, RA_mc, i_mc, theta_mc_t, theta_dot_GMO);

n3 = [0;0;1];
dr = r_mc_t - r_dc_t;
r1 = -dr/norm(dr);   % Since -r1 should point toward the mother craft
r2 = cross(dr,n3)/norm(cross(dr,n3));
r3 = cross(r1,r2);
 
RcN = [r1';r2';r3'];
rotation = RcN;

end