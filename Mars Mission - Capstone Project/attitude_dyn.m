function xdot = attitude_dyn(x, u, I_dc)
    sigma = x(1:3);
    w = x(4:6);

    sigma_dot = 0.25 * B_sigma(sigma) * w;
    w_dot = I_dc \ (-tilde(w)*I_dc*w + u);

    xdot = [sigma_dot; w_dot];
end