function [RN, w_RN_N, mode] = getReferenceFrame(t)

    % Daughter-Craft
    h_dc = 400*1000; % m
    R_mars = 3396.19*1000; % m
    r_LMO = R_mars + h_dc; % m

    mio_mars = 42828.3*10^9; % m^3/s^2
    theta_dot_LMO = sqrt(mio_mars / (r_LMO^3));  % rad/s

    RA_dc = 20 * pi/180; % DC right ascention
    i_dc = 30 * pi/180;  % DC Inclination
    theta_dc_0 = 60 * pi/180; % DC initial angle

    % Mother-Craft
    r_GMO = 20424.2*1000; % m
    theta_dot_GMO = sqrt(mio_mars / (r_GMO^3));  % rad/s

    RA_mc = 0;
    i_mc = 0;
    theta_mc_0 = 250 * pi/180; % MC initial angle

    [r_LMO_N, ~] = orbit_state(r_LMO, RA_dc, i_dc, theta_dc_0 + theta_dot_LMO*t, theta_dot_LMO);
    [r_GMO_N, ~] = orbit_state(r_GMO, RA_mc, i_mc, theta_mc_0 + theta_dot_GMO*t, theta_dot_GMO);

    if r_LMO_N(2) > 0
        RN = RsN_DCM();
        w_RN_N = [0;0;0];
        mode = "sun";
    else
        ang = acos(dot(r_LMO_N, r_GMO_N)/(norm(r_LMO_N)*norm(r_GMO_N)));

        if ang < 35*pi/180
            RN = RcN_DCM(t);
            w_RN_N = RcN_omega(t);
            mode = "gmo";
        else
            RN = RnN_DCM(t);
            HN = orbit_frame_dcm(t);
            w_RN_N = HN' * [0;0;theta_dot_LMO];
            mode = "nadir";
        end
    end
end