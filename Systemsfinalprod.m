function results = Systemsfinalprod(user_options)
clc;
% If the caller provided an options struct, use it. Otherwise start fresh.
if nargin > 0
    opts = user_options;
else
    opts = struct();
end

function params = default_parameters()
    params = struct();
    params.encoder.counts_per_rev = 500 * 4;
    params.max_encoder = 4096;
    params.motor.R  = 0.9;
    params.motor.L  = 0.45e-3;
    params.gear.N   = 21*15;
    params.vehicle.mass = 720;
    params.vehicle.Iz   = 1200;
    params.vehicle.a    = 1.72;
    params.vehicle.b    = 1.25;
    params.vehicle.Cf   = 100000;
    params.vehicle.Cr   = 100000;
    params.vehicle.Kt = 0.0259;
    params.tau = 0.126; %found by running a ton of simulations at a constant voltage
end

opts.Vel               = get_option(opts, 'Vel', 8);
opts.Ts                = get_option(opts, 'Ts', 0.001);
opts.id_duration       = get_option(opts, 'id_duration', 0.5);
opts.id_voltage        = get_option(opts, 'id_voltage', 63);

params = default_parameters();
if isfield(opts, 'params')
    params = merge_structs(params, opts.params);
end

%% Identification: collect step response from p-code

clear collect_pcode_response;
id_time = (0:opts.Ts:opts.id_duration)'; % Time vector for identification
const_volts=zeros(numel(id_time),1);
const_volts(:) = opts.id_voltage;
const_data = collect_pcode_response(const_volts, id_time, opts, params);

const_rack_model = steering(params, const_data);

%% Build the yaw-rate bicycle model and cascade with rack model
[yaw_tf] = build_bicycle_model(params, opts.Vel);
const_voltage_to_yaw_model = series(const_rack_model, yaw_tf);

%% Plots

figure('Name','Rack Identification');
plot(id_time, const_data.motor_angle_deg, 'b','DisplayName','p-code'); hold on;
step(const_rack_model,opts.id_duration);
grid on; xlabel('Time [s]'); ylabel('Steer Angle [deg]');
legend('show'); title('Steering Rack Model');

figure('Name', 'Yaw Rate Comparison constant input');
plot(id_time, const_data.yaw_rate, 'r', 'DisplayName', 'Yaw Rate'); hold on;
step(const_voltage_to_yaw_model,opts.id_duration);
grid on; xlabel('Time [s]'); ylabel('Yaw Rate [Deg/s]');
legend('show'); title('Yaw Rate Comparison');

figure('Name','Bode');
bode(const_voltage_to_yaw_model);
figure('Name','Eigenvalues');
p = pole(const_voltage_to_yaw_model);
plot(p, 0 , 'X', 'MarkerSize',10, 'LineWidth',3);
xlabel('Real'); ylabel('Imag');
title('Poles / Eigenvalues');

%% Bode Plot Verification

% frequencies to test (rad/s)
w_test = [0.5 1 2 5 10];  

% simulation settings
Ts = 0.001;
Tend = 20;                 % long enough to reach steady state
t = (0:Ts:Tend)';

% preallocate
gain_meas  = zeros(size(w_test));
phase_meas = zeros(size(w_test));

for k = 1:length(w_test)
    w = w_test(k);

    % input sine (amplitude 1)
    u = sin(w*t);

    % simulate
    y = lsim(const_voltage_to_yaw_model, u, t);

    % ignore transient: use last 30% of data
    idx0 = round(0.7*length(t));
    tt = t(idx0:end);
    uu = u(idx0:end);
    yy = y(idx0:end);

    % fit steady-state output to A*sin(wt + phi)
    % use linear regression on sin and cos terms:
    M = [sin(w*tt) cos(w*tt)];
    ab = M \ yy;                 % least squares
    A = hypot(ab(1), ab(2));     % amplitude
    phi = atan2(ab(2), ab(1));   % phase (rad), since A*sin+ B*cos form

    gain_meas(k)  = A;           % input amplitude is 1
    phase_meas(k) = phi;         % relative to sin(wt)
end

% Bode predicted values at same frequencies
[mag_bode, phase_bode] = bode(const_voltage_to_yaw_model, w_test);
mag_bode   = squeeze(mag_bode);            % magnitude
phase_bode = squeeze(phase_bode) * pi/180; % to rad

% plot comparison: magnitude
figure('Name','Stress Test');
subplot(2,1,1);
semilogx(w_test, 20*log10(mag_bode), 'o-', 'LineWidth', 1.5); hold on;
semilogx(w_test, 20*log10(gain_meas), 'x--', 'LineWidth', 1.5);
grid on;
xlabel('\omega (rad/s)'); ylabel('Magnitude (dB)');
title('Bode Magnitude vs Sine-Test Measured');
legend('Bode', 'Measured');

% plot comparison: phase
subplot(2,1,2)
semilogx(w_test, phase_bode*180/pi, 'o-', 'LineWidth', 1.5); hold on;
semilogx(w_test, phase_meas*180/pi, 'x--', 'LineWidth', 1.5);
grid on;
xlabel('\omega (rad/s)'); ylabel('Phase (deg)');
title('Bode Phase vs Sine-Test Measured');
legend('Bode', 'Measured');

end


%%%%%%%%%%%%%%%%%%%
%%% Local Functions
%%%%%%%%%%%%%%%%%%%

function merged = merge_structs(base, override)
    merged = base;
    names = fieldnames(override);
    for k = 1:numel(names)
        f = names{k};
        if isstruct(override.(f))
            merged.(f) = merge_structs(base.(f), override.(f));
        else
            merged.(f) = override.(f);
        end
    end
end

function val = get_option(opts, name, default)
    if isfield(opts, name)
        val = opts.(name);
    else
        val = default;
    end
end