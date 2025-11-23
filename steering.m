function G = steering(params, data)
    s = tf('s');
    N = params.gear.N;
    Kt = params.vehicle.Kt;
    Je = data.Je;
    Be = data.Be;

    G = (N*Kt)/(Je*s^2 + Be*s);
end