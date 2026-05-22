function w_RcN = RcN_omega(t)

dt = 1;
RcN = RcN_DCM(t);
RcN_next = RcN_DCM(t+dt);

RcN_dot = (1/dt)*(RcN_next - RcN);

w_tilde = -RcN_dot*transpose(RcN);

w1 = w_tilde(3,2);
w2 = w_tilde(1,3);
w3 = w_tilde(2,1);

w_RcN = [w1;w2;w3];
end