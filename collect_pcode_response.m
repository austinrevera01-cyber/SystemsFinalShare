function data = collect_pcode_response(voltage, time, opts, params)
    steps = numel(time);
    data.time     = time(:);
    data.voltage  = voltage(:);
    data.yaw_rate = zeros(steps,1);
    data.motor_angle_counts = zeros(steps,1);
    
    acc_counts   = 0;          
    last_raw     = NaN;         
    theta_counts = zeros(steps, 1);

    [~, ~, ~] = run_Indy_car_Fall_25(0,0,[0 0 0 0 0],0);

    clear run_Indy_car_Fall_25;

    for k = 1:steps
        [~, gyro_k, counts] = run_Indy_car_Fall_25(voltage(k), opts.Vel);
        data.yaw_rate(k)          = rad2deg(gyro_k);

        if isnan(last_raw)
            acc_counts = counts;
        else
            delta = counts - last_raw;
            if delta >  params.max_encoder / 2, delta = delta - params.max_encoder; end
            if delta < -params.max_encoder / 2, delta = delta + params.max_encoder; end
            acc_counts = acc_counts + delta;
        end
        last_raw = counts;
        theta_counts(k) = acc_counts;
    end

   data.motor_angle_rad = data.motor_angle_counts;

   %motor counts
   theta_m = (theta_counts * (2*pi / params.encoder.counts_per_rev));
   %motor to steering
   steering_degree = rad2deg(theta_m / params.gear.N);
   data.motor_angle_deg = steering_degree;
   
   data.Be = params.vehicle.Kt * params.gear.N * .35 / voltage(1);
   data.Je = params.tau * data.Be;
   
end