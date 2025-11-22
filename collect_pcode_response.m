function data = collect_pcode_response(voltage, time, Vel, params)
    steps = numel(time);
    data.time     = time(:);
    data.voltage  = voltage(:);
    data.yaw_rate = zeros(steps,1);
    data.motor_angle_counts = zeros(steps,1);


    for k = 1:steps
        [~, gyro_k, count_k] = run_Indy_car_Fall_25(voltage(k), Vel);
        data.yaw_rate(k)          = gyro_k;
        data.motor_angle_counts(k) = count_k;
    end
   data.motor_angle_rad
end