function HN_DCM = orbit_frame_dcm(t)
h_dc = 400*1000; % m
R_mars = 3396.19*1000; % m
r_LMO = R_mars + h_dc; % m

mio_mars = 42828.3*10^9; % m^3/s^2
theta_dot_LMO = sqrt(mio_mars / (r_LMO^3));  % rad/s

RA_dc = 20 * pi/180; % DC right ascention
i_dc = 30 * pi/180;  % DC Inclination
theta_dc_0 = 60 * pi/180; % DC initial angle

theta_dc_t = rem(theta_dot_LMO*t + theta_dc_0, 2*pi);
[r_dc_t, r_dot_dc_t] = orbit_state(r_LMO, RA_dc, i_dc, theta_dc_t, theta_dot_LMO);


i_r = r_dc_t / norm(r_dc_t);
i_h = cross(r_dc_t, r_dot_dc_t) / norm(cross(r_dc_t, r_dot_dc_t));

i_theta = cross(i_h, i_r);

HN_DCM = [i_r';i_theta';i_h'];
end