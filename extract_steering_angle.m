%EXTRACT_STEERING_ANGLE Helper to call run_Indy_car_Fall_25 and compute wheel steer angle.
%   [wheel_angle_rad, info] = EXTRACT_STEERING_ANGLE(volts, Vel, X0_values, WP_FILE)
%   executes the compiled P-code model (run_Indy_car_Fall_25.p) and converts the
%   returned motor encoder counts into a wheel steering angle at the tire.
%
%   Inputs mirror the P-code signature:
%       volts      - Steering motor voltage command [V]
%       Vel        - Vehicle speed [m/s] (optional)
%       X0_values  - Initial state vector [East North Heading yaw_rate motor_angle] (optional)
%       WP_FILE    - Waypoint selector (optional)
%
%   Outputs:
%       wheel_angle_rad - Wheel steering angle in radians (positive = left turn)
%       info            - Struct with intermediate conversions:
%                           .motor_counts   -> Encoder counts returned by P-code
%                           .motor_rad      -> Motor shaft angle [rad]
%                           .counts_per_rev -> Counts per motor revolution used
%                           .rack_per_rad   -> Rack travel per motor rad [m/rad]
%                           .rack_to_wheel  -> Wheel angle per meter of rack [rad/m]
%
%   Defaults for isolated runs:
%       volts = 12 V, Vel = 8 m/s, X0_values = default (per run_Indy_car_Fall_25), WP_FILE = default.
%
%   Example:
%       % Compute steering angle after a 6 V command at 20 m/s using default X0 and waypoints
%       [steer_angle, details] = extract_steering_angle(6, 20);
%       fprintf('Wheel angle: %.3f rad (%.2f deg)\n', steer_angle, rad2deg(steer_angle));
%
%       % Run in isolation with defaults (12 V, 8 m/s)
%       [steer_angle_default, details_default] = extract_steering_angle();
%
%   Notes:
%       - Encoder counts are assumed to be 500 CPT with 4x quadrature (2000 counts/rev).
%       - The geometric conversion uses the same rack/wheel mapping as the yaw model.
%       - This helper intentionally isolates P-code execution so downstream scripts can
%         request only the steering angle without unpacking all other outputs.
%
%   Author: GPT-5.1-Codex-Max (ChatGPT)
%   Date:   2025-XX-XX

function [wheel_angle_rad, info] = extract_steering_angle(volts, Vel, X0_values, WP_FILE)

% Conversion constants derived from project documentation
COUNTS_PER_REV = 2000;       % 500 CPT encoder with 4x quadrature
RACK_PER_RAD   = 0.003;      % rack travel per motor rad [m/rad]
RACK_TO_WHEEL  = 70;         % wheel angle per meter of rack [rad/m]

volts = 12;  % default steering motor voltage [V]
Vel = 8;     % default vehicle speed [m/s]

[~, ~, motor_counts] = run_Indy_car_Fall_25(volts,Vel);

% Convert encoder counts -> motor shaft angle [rad]
motor_rad = (motor_counts * 2 * pi) / COUNTS_PER_REV;
wheel_angle_rad = motor_rad * RACK_PER_RAD * RACK_TO_WHEEL;

% Collect supporting details
if nargout > 1
    info = struct();
    info.motor_counts = motor_counts;
    info.motor_rad = motor_rad;
    info.counts_per_rev = COUNTS_PER_REV;
    info.rack_per_rad = RACK_PER_RAD;
    info.rack_to_wheel = RACK_TO_WHEEL;
    info.inputs = struct('volts', volts, 'Vel', Vel, ...
                         'X0_values', X0_values, 'WP_FILE', WP_FILE);
end

end