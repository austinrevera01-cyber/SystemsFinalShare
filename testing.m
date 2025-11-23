close all; clc; clear;

vstep = 8;
vel = 0;
cpr = 2000;
maxcount = 4096;
N = 21*15;
dt = 0.001;
t_end = .5;
t = 0:dt:t_end;
steps = numel(t);
steering_max = 20;
kt = 0.0259;

acc_counts   = 0;          
last_raw     = NaN;         
theta_counts = zeros(steps, 1);

[~, ~, ~] = run_Indy_car_Fall_25(0,0,[0 0 0 0 0],0);

clear run_Indy_car_Fall_25;

for k = 1:steps
    [~, ~, counts] = run_Indy_car_Fall_25(vstep,vel);

    if isnan(last_raw)
        acc_counts = counts;
    else
        delta = counts - last_raw;
        if delta >  maxcount / 2, delta = delta - maxcount; end
        if delta < -maxcount / 2, delta = delta + maxcount; end
        acc_counts = acc_counts + delta;
    end

    last_raw = counts;
    theta_counts(k) = acc_counts;
end

%motor counts
theta_m = (theta_counts * (2*pi / cpr));
%motor to steering
steering_degree = (theta_m / N);

s = tf('s');
[~,idx] = max(steering_degree);
derivative = gradient(steering_degree, t);
SS = max(derivative);
tau = t(idx)/5;
gain = SS/vstep;
Be= N*kt/gain;
Je = tau * Be;

% Plot the steering degree over time
figure;
plot(t, steering_degree);hold on;
xlabel('Time (s)');
ylabel('Steering Degree (°)');
title('Steering Degree vs Time');
grid on;hold on;
%step((N*kt)/(Je*s^2 + Be*s), t);
% Calculate the derivative of the steering degree

% Plot the derivative of the steering degree over time
figure;
plot(t, derivative);
xlabel('Time (s)');
ylabel('Rate of Change of Steering Degree (°/s)');
title('Rate of Change of Steering Degree vs Time');
grid on; hold on
%step((N*kt)/(Je*s + Be), t);