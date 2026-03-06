function rotation = RnN_DCM(t)

RnH = M1(pi)*M3(pi);
HN = orbit_frame_dcm(t);

rotation = RnH*HN;
end