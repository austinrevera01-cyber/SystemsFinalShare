function plots(time, voltage, data, rack_model, val)
    figure('Name','Rack Identification');
    subplot(2,1,1);
    plot(time, voltage,'LineWidth',1.2);
    grid on; ylabel('Voltage [V]');
    title('Identification Input');

    subplot(2,1,2);
    plot(time, data.motor_angle_rad, 'b','DisplayName','p-code'); hold on;
    %plot(time, lsim(rack_model, voltage, time), 'r--','DisplayName','1st-order fit');
    grid on; xlabel('Time [s]'); ylabel('Steer Angle [rad]');
    legend('show'); title('Steering Rack Fit');

    figure('Name','Validation – Step Input');
    subplot(2,1,1);
    plot(val.step.time, val.step.input_voltage,'LineWidth',1.2);
    grid on; ylabel('Voltage [V]');
    title('Step Input');

    subplot(2,1,2);
    plot(val.step.time, val.step.ref_yaw_rate,'b','DisplayName','p-code'); hold on;
    plot(val.step.time, val.step.model_yaw_rate,'r--','DisplayName','Model');
    grid on; xlabel('Time [s]'); ylabel('Yaw Rate [rad/s]');
    legend('show');
    title(sprintf('Step Response RMSE = %.4f', val.step.rmse));

    figure('Name','Validation – Chirp Input');
    subplot(2,1,1);
    plot(val.chirp.time, val.chirp.input_voltage,'LineWidth',1.2);
    grid on; ylabel('Voltage [V]');
    title('Chirp Input');

    subplot(2,1,2);
    plot(val.chirp.time, val.chirp.ref_yaw_rate,'b','DisplayName','p-code'); hold on;
    plot(val.chirp.time, val.chirp.model_yaw_rate,'r--','DisplayName','Model');
    grid on; xlabel('Time [s]'); ylabel('Yaw Rate [rad/s]');
    legend('show');
    title(sprintf('Chirp Response RMSE = %.4f', val.chirp.rmse));
end