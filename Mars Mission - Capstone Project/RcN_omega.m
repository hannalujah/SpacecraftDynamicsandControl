function w_RcN = RcN_omega(t)

dt = 0.1;
RcN = RcN_DCM(t);
RcN_prev = RcN_DCM(t-dt);
RcN_next = RcN_DCM(t+dt);

RcN_dot = RcN_next - RcN_prev / (2*dt);

w_tilde = RcN_dot*transpose(RcN);

w1 = -w_tilde(2,3);
w2 = w_tilde(1,3);
w3 = -w_tilde(1,2);

w_RcN = [w1;w2;w3];
end