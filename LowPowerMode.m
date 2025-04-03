clc;
clear;
close all;

% --- CubeSat Parameters ---
% Deployable solar panels
A_deploy_7 = 0.0182; % Area of deployable panel with 7 solar panels (m^2)
A_deploy_4 = 0.0104; % Area of deployable panel with 4 solar panels (m^2)
A_deploy_2 = 0.0052; % Area of deployable panel with 2 solar panels (m^2)
eta_7 = 0.267; % Efficiency of panels with 7 cells
eta_4 = 0.267; % Efficiency of panels with 4 cells
eta_2 = 0.267; % Efficiency of panels with 2 cells

G = 1380.69; % Solar irradiance in space (W/m^2)

% --- Orbital Parameters ---
altitude = 500e3; % Altitude of CubeSat (500 km)
Re = 6371e3; % Earth's radius (m)
T_orbit = 2 * pi * sqrt((Re + altitude)^3 / 3.986e14); % Orbital period (s)
t = linspace(0, T_orbit, 1000); % Time vector
omega = 2 * pi / T_orbit; % Orbital angular velocity (rad/s)

% --- Battery Parameters ---
battery_capacity = 100; % Battery capacity (Wh)
battery_level = battery_capacity * 0.8; % Start at 80%
charge_efficiency = 0.9; % Charging losses
discharge_efficiency = 1.1; % Discharge losses

% --- Sun Vector (Earth-Centered Inertial) ---
i = 98.6; % Inclination angle (sun-synchronous)
dec = 23.5; % Sun declination angle (Earth axial tilt)

% --- Power Generation Initialization ---
P_gen_body = zeros(size(t));
P_gen_deploy_7 = zeros(size(t));
P_gen_deploy_4 = zeros(size(t));
P_gen_deploy_2 = zeros(size(t));
P_total = zeros(size(t));

% --- CubeSat Fixed Tilt Angle ---
tilt_angle = 98.73; % Tilt angle of CubeSat (degrees)
R_tilt = [cosd(tilt_angle) 0 sind(tilt_angle); 0 1 0; -sind(tilt_angle) 0 cosd(tilt_angle)];

% --- Attitude Dynamics ---
omega_roll = 2 * pi / (T_orbit / 4);
omega_pitch = 2 * pi / (T_orbit / 3);
omega_yaw = 2 * pi / (T_orbit / 2);

for k = 1:length(t)
    theta = omega * t(k);

    Sun_ECI = [cos(0) * cosd(dec); sind(0) * cosd(dec); sind(dec)];
    Sun_ECI = Sun_ECI / norm(Sun_ECI);

    R_roll = [1 0 0; 0 cos(omega_roll * t(k)) -sin(omega_roll * t(k)); 0 sin(omega_roll * t(k)) cos(omega_roll * t(k))];
    R_pitch = [cos(omega_pitch * t(k)) 0 sin(omega_pitch * t(k)); 0 1 0; -sin(omega_pitch * t(k)) 0 cos(omega_pitch * t(k))];
    R_yaw = [cos(omega_yaw * t(k)) -sin(omega_yaw * t(k)) 0; sin(omega_yaw * t(k)) cos(omega_yaw * t(k)) 0; 0 0 1];

    R_attitude = R_roll * R_pitch * R_yaw * R_tilt;

    Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    Sun_body = R_attitude * (Rz * Sun_ECI);

    P_gen_deploy_7(k) = 2 * A_deploy_7 * G * eta_7 * max(0, Sun_body(1));
    P_gen_deploy_4(k) = 2 * A_deploy_4 * G * eta_4 * max(0, Sun_body(1));
    P_gen_deploy_2(k) = 4 * A_deploy_2 * G * eta_2 * max(0, Sun_body(1));

    P_total(k) = P_gen_deploy_7(k) + P_gen_deploy_4(k) + P_gen_deploy_2(k);
end

% --- Sunlight and Eclipse Periods ---
eclipse_fraction = 0.3778; % of orbit in eclipse
t_sunlight = T_orbit * (1 - eclipse_fraction);
t_eclipse = T_orbit * eclipse_fraction;

% Create Sunlight-Eclipse Profile
sunlight_flag = ones(size(t)); 
sunlight_flag(t > t_sunlight) = 0; % Eclipse portion

% --- Power Consumption Initialization ---
P_consumed = zeros(size(t));
smooth_factor = 0.05; % Smoothing factor
previous_power = 15; % Initial power level

for k = 1:length(t)
    % Base power loads (always on)
    obc_power = 1.0; % OBC (On-Board Computer)
    passive_power = 1.0; % Passive loads

    % ADCS Power
    adcs_power = 4.0 + 0.5 * rand();  % ADCS active

    % Dynamic loads with duty cycle control
    if sunlight_flag(k) == 1  % During Sunlight
        payload_power = 2.0 * mod(floor(t(k) / 300), 2);  % On for 5 min, off for 5 min
        uhf_transmit_power = 8.0 * (mod(floor(t(k) / 600), 2) == 0); % Transmit every 10 min
        x_band_transmit_power = 10.0 * (mod(floor(t(k) / 1200), 2) == 0); % Transmit every 20 min
    else  % During Eclipse
        payload_power = 0;  
        uhf_transmit_power = 1;  % Low-power beacon mode
        x_band_transmit_power = 7.0; % Reduced power
        active_thermal_power = 0;  % No thermal heating in eclipse
    end

    % During Eclipse: Apply Power Reduction
    if P_total(k) == 0  
        eclipse_reduction_factor = 0.5 + 0.25 * rand();  % Random 30-50% reduction
        obc_power = obc_power * eclipse_reduction_factor;
        passive_power = passive_power * eclipse_reduction_factor;
        adcs_power = adcs_power * eclipse_reduction_factor;
        uhf_transmit_power = uhf_transmit_power * eclipse_reduction_factor;
        x_band_transmit_power = x_band_transmit_power * eclipse_reduction_factor;
    end

    % Total Power Consumption
    P_temp = obc_power + adcs_power + passive_power + payload_power + uhf_transmit_power + x_band_transmit_power;
    
    % Smooth Transitions
    P_consumed(k) = smooth_factor * P_temp + (1 - smooth_factor) * previous_power;
    previous_power = P_consumed(k);

    % Battery charge/discharge update
    power_balance = P_total(k) - P_consumed(k);
    energy_change = power_balance * (T_orbit / length(t)) / 3600;  % Convert W to Wh
    
    if power_balance > 0
        battery_level = min(battery_capacity, battery_level + energy_change * charge_efficiency);
    else
        battery_level = max(0, battery_level + energy_change * discharge_efficiency);
    end
    
    battery_energy(k) = battery_level; 
end

% --- Plot Power Data ---
figure;
subplot(3,1,1);
plot(t/60, P_consumed, 'r', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Consumption');
grid on;

max_body = max(P_consumed);
min_body = min(P_consumed);
avg_body = mean(P_consumed);
annotation('textbox', [0.75, 0.85, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_body, min_body, avg_body), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');

subplot(3,1,2);
plot(t/60, P_total, 'b', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Generation');
grid on;

max_deploy_7 = max(P_total);
min_deploy_7 = min(P_total);
avg_deploy_7 = mean(P_total);
annotation('textbox', [0.75, 0.55, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_deploy_7, min_deploy_7, avg_deploy_7), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');

subplot(3,1,3);
plot(t/60, battery_energy, 'g', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Battery Energy (Wh)');
title('Battery State of Charge');
grid on;

max_deploy_4 = max(battery_energy);
min_deploy_4 = min(battery_energy);
avg_deploy_4 = mean(battery_energy);
annotation('textbox', [0.75, 0.25, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f \nMin: %.2f \nAve: %.2f ', max_deploy_4, min_deploy_4, avg_deploy_4), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');

figure;
yyaxis left
plot(t/60, P_consumed, 'r', 'LineWidth', 1.5);
hold on;
plot(t/60, P_total, 'b', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Generation, Consumption, and Battery Energy of DSA3C');
legend('Power Consumed', 'Power Generated');
grid on;

yyaxis right
plot(t/60, battery_energy, 'g', 'LineWidth', 1.5);
ylabel('Battery Energy (Wh)');
legend('Power Consumed', 'Power Generated', 'Battery Energy');
hold off;

disp(['Max Power Consumed: ', num2str(max(P_consumed)), ' W']);
disp(['Min Power Consumed: ', num2str(min(P_consumed)), ' W']);
