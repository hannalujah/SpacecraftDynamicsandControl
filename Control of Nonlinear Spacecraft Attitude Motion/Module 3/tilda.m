function vec_tilda = tilda(v)

vec_tilda = [0, -v(3), v(2);
        v(3), 0, -v(1);
        -v(2), v(1), 0];

end