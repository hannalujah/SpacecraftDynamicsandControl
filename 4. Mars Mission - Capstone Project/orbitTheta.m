function theta = orbitTheta(time,thetaDot,theta0)

theta = rem(thetaDot*time+theta0, 2*pi);

end

