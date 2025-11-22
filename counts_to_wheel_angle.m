function theta = counts_to_wheel_angle(counts, params)
    motor_rad = counts * (2*pi / params.encoder.counts_per_rev);
    theta = motor_rad / params.gear.N;
    theta = wrap_angle(theta, [0 .36]);
end