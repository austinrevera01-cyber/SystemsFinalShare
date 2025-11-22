function G = steering(params)
    s = tf('s');
    J = params.motor.J;
    B = params.motor.B;
    N = params.gear.N;
    Kt = params.vehicle.Kt;

    G = (N*Kt)/(J*s^2 + B*s);
end