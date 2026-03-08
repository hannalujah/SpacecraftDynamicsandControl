function [RN, w_RN_N, mode] = getReference(t, modeName)
    % Daughter-Craft
    h_dc = 400*1000; % m
    R_mars = 3396.19*1000; % m
    r_LMO = R_mars + h_dc; % m

    mio_mars = 42828.3*10^9; % m^3/s^2
    theta_dot_LMO = sqrt(mio_mars / (r_LMO^3));  % rad/s

    switch modeName
        case "sun"
            RN = RsN_DCM();
            w_RN_N = [0;0;0];
            mode = "sun";

        case "nadir"
            RN = RnN_DCM(t);
            HN = orbit_frame_dcm(t);
            w_RN_N = HN' * [0;0;theta_dot_LMO];
            mode = "nadir";

        case "gmo"
            RN = RcN_DCM(t);
            w_RN_N = RcN_omega(t);
            mode = "gmo";
    end
end