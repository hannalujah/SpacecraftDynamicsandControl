function x_next = rk4_step(x, u, dt, I_dc)
    k1 = attitude_dyn(x, u, I_dc);
    k2 = attitude_dyn(x + 0.5*dt*k1, u, I_dc);
    k3 = attitude_dyn(x + 0.5*dt*k2, u, I_dc);
    k4 = attitude_dyn(x + dt*k3, u, I_dc);

    x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    sigma = x_next(1:3);
    if norm(sigma) > 1
        x_next(1:3) = sigmaToSigmaS(sigma);
    end
end