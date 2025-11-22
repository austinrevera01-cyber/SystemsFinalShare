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
    params.motor.R  = 0.9;
    params.motor.L  = 0.45e-3;
    params.motor.J  = 0.000001;
    params.motor.B  = 2.36e-5;
    params.gear.N   = 1/(21*15);
    params.vehicle.mass = 720;
    params.vehicle.Iz   = 1200;
    params.vehicle.a    = 1.72;
    params.vehicle.b    = 1.25;
    params.vehicle.Cf   = 100000;
    params.vehicle.Cr   = 100000;
    params.vehicle.Kt = 0.000001;
end

opts.Vel               = get_option(opts, 'Vel', 8);
opts.Ts                = get_option(opts, 'Ts', 0.001);
opts.id_duration       = get_option(opts, 'id_duration', 1.5);
opts.val_duration      = get_option(opts, 'val_duration', 2);
opts.id_voltage        = get_option(opts, 'id_voltage', 12);
opts.val_step_voltage  = get_option(opts, 'val_step_voltage', 6);
opts.val_chirp_voltage = get_option(opts, 'val_chirp_voltage', 4);
opts.chirp_f0          = get_option(opts, 'chirp_f0', 0.2);
opts.chirp_f1          = get_option(opts, 'chirp_f1', 2.0);
opts.pre_duration      = get_option(opts, 'pre_duration', 0.2);

params = default_parameters();
if isfield(opts, 'params')
    params = merge_structs(params, opts.params);
end

%% Identification: collect step response from p-code
[id_time, id_voltage] = step_profile(opts.id_voltage, opts.id_duration, opts.Ts, opts.pre_duration);

clear collect_pcode_response;
id_data = collect_pcode_response(id_voltage, id_time, opts.Vel, params);

% Fit a simple first-order transfer function for the steering rack
rack_model = steering(params);

%% Build the yaw-rate bicycle model and cascade with rack model
[yaw_tf] = build_bicycle_model(params, opts.Vel);
voltage_to_yaw_model = series(rack_model, yaw_tf);

% Store everything neatly
results = struct();
results.rack_model           = rack_model;
results.yaw_model            = yaw_tf;
results.voltage_to_yaw_model = voltage_to_yaw_model;
bode(voltage_to_yaw_model)

%% Validation #1 — Step input
[val_step_time, val_step_voltage] = step_profile(opts.val_step_voltage, ...
                                                 opts.val_duration, ...
                                                 opts.Ts, ...
                                                 opts.pre_duration);

val_step_sim = lsim(voltage_to_yaw_model, val_step_voltage, val_step_time);
clear collect_pcode_response;
val_step_ref = collect_pcode_response(val_step_voltage, val_step_time, opts.Vel, params);

results.validation.step.time           = val_step_time;
results.validation.step.input_voltage  = val_step_voltage;
results.validation.step.model_yaw_rate = val_step_sim;
results.validation.step.ref_yaw_rate   = val_step_ref.yaw_rate;
results.validation.step.rmse           = rms(val_step_sim - val_step_ref.yaw_rate);

%% Validation #2 — Chirp input
[val_chirp_time, val_chirp_voltage] = chirp_profile(opts.val_chirp_voltage, ...
                                                     opts.val_duration, ...
                                                     opts.Ts, ...
                                                     opts.chirp_f0, ...
                                                     opts.chirp_f1);

val_chirp_sim = lsim(voltage_to_yaw_model, val_chirp_voltage, val_chirp_time);
clear collect_pcode_response;
val_chirp_ref = collect_pcode_response(val_chirp_voltage, val_chirp_time, opts.Vel, params);

results.validation.chirp.time           = val_chirp_time;
results.validation.chirp.input_voltage  = val_chirp_voltage;
results.validation.chirp.model_yaw_rate = val_chirp_sim;
results.validation.chirp.ref_yaw_rate   = val_chirp_ref.yaw_rate;
results.validation.chirp.rmse           = rms(val_chirp_sim - val_chirp_ref.yaw_rate);

%% Plots
plots(id_time, id_voltage, id_data, rack_model,results.validation);

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

function [t, u] = step_profile(U, duration, Ts, pre)
    if nargin < 4
        pre = 0.2;
    end
    n_pre  = round(pre / Ts);
    n_main = round(duration / Ts) + 1;
    u = [zeros(n_pre,1); U * ones(n_main,1)];
    t = (0:length(u)-1)' * Ts;
end

function [t, u] = chirp_profile(A, duration, Ts, f0, f1)
    t = (0:Ts:duration)';
    u = A * chirp(t, f0, duration, f1);
end