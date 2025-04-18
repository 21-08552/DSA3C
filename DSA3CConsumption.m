clc;
clear;
close all;

% --- CubeSat Parameters ---
% Deployable solar panels
A_deploy_7 = 0.0182; % Area of deployable panel with 7 solar panels (m^2)
A_deploy_4 = 0.0104; % Area of deployable panel with 4 solar panels (m^2)
A_deploy_2 = 0.0052; % Area of deployable panel with 2 solar panels (m^2)
eta_7 = 0.307; % Efficiency of panels with 7 cells
eta_4 = 0.307; % Efficiency of panels with 4 cells
eta_2 = 0.307; % Efficiency of panels with 2 cells

G = 1380.69; % Solar irradiance in space (W/m^2)
L = 0.3 * 0.082 * cosd(90); % Length of deployable panel (m)

% --- Orbital Parameters ---
altitude = 500e3; % Altitude of CubeSat (500 km)
Re = 6371e3; % Earth's radius (m)
T_orbit = 2*pi*sqrt((Re+altitude)^3/3.986e14); % Orbital period (s)
t = linspace(0, T_orbit, 1000); % Time vector
omega = 2*pi/T_orbit; % Orbital angular velocity (rad/s)


% Battery Parameters
battery_capacity = 100; % Battery capacity (Wh)
battery_level = battery_capacity * 0.8; % Start at 80%
charge_efficiency = 0.9; % Charging losses
discharge_efficiency = 1.1; % Discharge losses

% --- Sun Vector (Earth-Centered Inertial) ---
i = 98.6; % Inclination angle (sun-synchronous)
dec = 23.5; % Sun declination angle (Earth axial tilt)

% Power generation initialization
P_gen_body = zeros(size(t));
P_gen_deploy_7 = zeros(size(t));
P_gen_deploy_4 = zeros(size(t));
P_total = zeros(size(t));

% --- CubeSat Fixed Tilt Angle ---
tilt_angle = 98.73; % Tilt angle of CubeSat (degrees)
R_tilt = [cosd(tilt_angle) 0 sind(tilt_angle); 0 1 0; -sind(tilt_angle) 0 cosd(tilt_angle)];

P_gen_deploy_7 = zeros(size(t));
P_gen_deploy_4 = zeros(size(t));
P_gen_deploy_2 = zeros(size(t));
P_total = zeros(size(t));

% --- Simulate CubeSat Attitude Dynamics ---
omega_roll = 2*pi/(T_orbit/4);
omega_pitch = 2*pi/(T_orbit/3);
omega_yaw = 2*pi/(T_orbit/2);

for k = 1:length(t)
    theta = omega * t(k);
    r = [cos(theta); sin(theta); 0];

    Sun_ECI = [cos(0) * cosd(dec); sind(0) * cosd(dec); sind(dec)];
    Sun_ECI = Sun_ECI / norm(Sun_ECI);

    R_roll = [1 0 0; 0 cos(omega_roll*t(k)) -sin(omega_roll*t(k)); 0 sin(omega_roll*t(k)) cos(omega_roll*t(k))];
    R_pitch = [cos(omega_pitch*t(k)) 0 sin(omega_pitch*t(k)); 0 1 0; -sin(omega_pitch*t(k)) 0 cos(omega_pitch*t(k))];
    R_yaw = [cos(omega_yaw*t(k)) -sin(omega_yaw*t(k)) 0; sin(omega_yaw*t(k)) cos(omega_yaw*t(k)) 0; 0 0 1];

    R_attitude = R_roll * R_pitch * R_yaw * R_tilt;

    Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    Sun_body = R_attitude * (Rz * Sun_ECI);

    P_gen_deploy_7(k) = 2 * A_deploy_7 * G * eta_7 * max(0, Sun_body(1));
    P_gen_deploy_4(k) = 2 * A_deploy_4 * G * eta_4 * max(0, Sun_body(1));
    P_gen_deploy_2(k) = 4 * A_deploy_2 * G * eta_2 * max(0, Sun_body(1));

    P_total(k) = P_gen_deploy_7(k) + P_gen_deploy_4(k) + P_gen_deploy_2(k);
end

% --- Sunlight and Eclipse Periods ---
eclipse_fraction = 0.3778; % 35% of orbit in eclipse
t_sunlight = T_orbit * (1 - eclipse_fraction);
t_eclipse = T_orbit * eclipse_fraction;

% Create Sunlight-Eclipse Profile
sunlight_flag = ones(size(t)); 
sunlight_flag(t > t_sunlight) = 0; % Eclipse portion

% --- CubeSat Power Consumption ---
P_consumed = zeros(size(t));
smooth_factor = 0.1; % Stronger smoothing
previous_power = 15; % Initial power level

for k = 1:length(t)
    % Base power loads (always on)
    obc_power = 1.0; % OBC (On-Board Computer)
    passive_power = 1.0; % Passive loads

    % Generate a random duty cycle between 80% and 100% for each cycle
    random_duty_cycle = 0.9 + (0.2 * rand()); % Random value between 0.8 and 1.0

    % Compute active duration based on this duty cycle
    cycle_duration = 900; % 15 minutes cycle
    active_duration = random_duty_cycle * cycle_duration;  

    % ADCS Power Update after Adding 4 Panels (20-30% increase)
    increase_factor = 1 + (0.2 + (0.1 * rand())); % Randomize 20-30%

    if mod(t(k), cycle_duration) < active_duration  
    adcs_power = (4.5 + rand()) * increase_factor; % Peak power
    else
    adcs_power = (1.5 + 0.5 * rand()) * increase_factor; % Baseline power
    end

    % Dynamic loads with duty cycle control
    if sunlight_flag(k) == 1  % During Sunlight
        payload_power = 2.0 * mod(floor(t(k) / 300), 2); % On for 5 minutes, off for 5 minutes
        uhf_transmit_power = 8.0 * (mod(floor(t(k) / 600), 2) == 0); % Transmit every 10 min
        x_band_transmit_power = 10.0 * (mod(floor(t(k) / 1200), 2) == 0); % Transmit every 20 min
    else  % During Eclipse
        payload_power = 0; 
        uhf_transmit_power = 1; % low-power beacon mode
        x_band_transmit_power = 7.0 * (mod(floor(t(k) / 1200), 2) == 0); % Reduced power in eclipse
         active_thermal_power = 0; % Lowered thermal load in eclipse
    end

    % During Eclipse (Passive Thermal System Consideration)
    if sunlight_flag(k) == 0  
       passive_thermal_power = -2.0; % Heat loss due to radiation  
    else
       passive_thermal_power = 0; % No heat loss in sunlight  
    end

     % Total power consumption at this time step
    P_temp = obc_power + adcs_power + passive_power + payload_power + uhf_transmit_power + x_band_transmit_power + passive_thermal_power;
    
    % Smooth transitions
    P_consumed(k) = smooth_factor * P_temp + (1 - smooth_factor) * previous_power;
    previous_power = P_consumed(k);
    
  % Battery charge/discharge update (in Wh)
    power_balance = P_total(k) - P_consumed(k); % Net power balance
    energy_change = power_balance * (T_orbit / length(t)) / 3600; % Convert W to Wh
    
    if power_balance > 0
        % Charging mode (limited by battery capacity)
        battery_level = min(battery_capacity, battery_level + energy_change * charge_efficiency);
    else
        % Discharging mode (cannot go below 0 Wh)
        battery_level = max(0, battery_level + energy_change * discharge_efficiency);
    end
    
    battery_energy(k) = battery_level; % Store battery energy in Wh
end

for k = 1:length(t)
    % Check if within 40-60 minutes
  if t(k)/60 >= 40 && t(k)/60 <= 70
    charge_efficiency = 0.95;  % Boost efficiency in this range
  else
    charge_efficiency = 0.9;  % Normal efficiency
  end
    % Compute power balance
    power_balance = P_total(k) - P_consumed(k);

    if (t(k)/60 >= 40) && (t(k)/60 <= 70)
    payload_power = payload_power * 0.8; % Reduce payload power by 20%
    uhf_transmit_power = uhf_transmit_power * 0.8; % Reduce UHF power
    end
battery_energy = zeros(size(t));  
battery_energy(1) = battery_capacity * 0.8;  

for k = 1:length(t)
    power_balance = P_total(k) - P_consumed(k);
    
    if k == 1
        battery_energy(k) = battery_energy(1);  
    else
        battery_energy(k) = max(0, battery_energy(k-1) + (power_balance * discharge_efficiency * (T_orbit / 1000) / 3600));
    end
end
    if (battery_energy(k) < 10) && (t(k)/60 >= 40) && (t(k)/60 <= 70)
    discharge_efficiency = 1.2;  % Increase discharge efficiency
    end
end


% --- Plot Power Consumption ---
figure;
plot(t/60, P_consumed, 'r', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Consumption of DSA3C');
grid on;

% Display stats neatly in the upper-right corner
max_total = max(P_consumed);
min_total = min(P_consumed);
avg_total = mean(P_consumed);
annotation('textbox', [0.75, 0.85, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_total, min_total, avg_total), ...
    'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'white');



% --- Display Results ---
disp(['Max Power Generated: ', num2str(max(P_total)), ' W']);
disp(['Min Power Generated: ', num2str(min(P_total)), ' W']);
disp(['Max Power Generated: ', num2str(max(P_consumed)), ' W']);
disp(['Max Power Generated: ', num2str(mean(P_consumed)), ' W']);