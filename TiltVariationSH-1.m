clc;
clear;
close all;

% --- CubeSat Parameters ---
% Deployable solar panels
A_deploy_7 = 0.0182; % Area of deployable panel with 7 solar panels (m^2)
A_deploy_4 = 0.0104; % Area of deployable panel with 4 solar panels (m^2)
eta_7 = 0.307; % Efficiency of panels with 7 cells
eta_4 = 0.307; % Efficiency of panels with 4 cells

% Body-mounted solar panels
A_body_side = 0.0104; % Area of each body-mounted panel (m^2)
eta_body = 0.307; % Efficiency of body-mounted panels

G = 1380.69; % Solar irradiance in space (W/m^2)
L = 0.3 * 0.082 * cos(90); % Length of deployable panel (m)

% --- Orbital Parameters ---
altitude = 500e3; % Altitude of CubeSat (500 km)
Re = 6371e3; % Earth's radius (m)
T_orbit = 2*pi*sqrt((Re+altitude)^3/3.986e14); % Orbital period (s)
t = linspace(0, T_orbit, 1000); % Time vector
omega = 2*pi/T_orbit; % Orbital angular velocity (rad/s)

% Battery Parameters
battery_capacity = 100; % Battery capacity (Wh)
battery_level = battery_capacity * 1; % Start at 80%
charge_efficiency = 0.9; % Charging losses
discharge_efficiency = 1.1; % Discharge losses


% --- Sun Vector (Earth-Centered Inertial) ---
i = 98.6; % Inclination angle (sun-synchronous)
dec = 23.5; % Sun declination angle (Earth axial tilt)

P_gen_body = zeros(size(t));
P_gen_deploy_7 = zeros(size(t));
P_gen_deploy_4 = zeros(size(t));
P_total = zeros(size(t));

% --- CubeSat Tilt Angles ---
tilt_x = 135;   % X-axis tilt angle in degrees
tilt_z = 1;   % Z-axis tilt angle in degrees
tilt_y = -30; % Y-axis tilt angle (original)

% Rotation matrices for tilts along X, Y, and Z
R_x = [1 0 0; 0 cosd(tilt_x) -sind(tilt_x); 0 sind(tilt_x) cosd(tilt_x)];
R_y = [cosd(tilt_y) 0 sind(tilt_y); 0 1 0; -sind(tilt_y) 0 cosd(tilt_y)];
R_z = [cosd(tilt_z) -sind(tilt_z) 0; sind(tilt_z) cosd(tilt_z) 0; 0 0 1];

% Combined tilt matrix
R_tilt = R_x * R_y * R_z;
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

    P_gen_body(k) = 2 * A_body_side * G * eta_body * max(0, Sun_body(3));
    P_gen_deploy_7(k) = 2 * A_deploy_7 * G * eta_7 * max(0, Sun_body(1));
    P_gen_deploy_4(k) = 2 * A_deploy_4 * G * eta_4 * max(0, Sun_body(1));

    P_total(k) = P_gen_body(k) + P_gen_deploy_7(k) + P_gen_deploy_4(k);
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
random_duty_cycle = 0.8 + (0.2 * rand()); % Random value between 0.8 and 1.0

% Compute active duration based on this duty cycle
cycle_duration = 900; % 15 minutes cycle
active_duration = random_duty_cycle * cycle_duration;  

% ADCS power variations using the random duty cycle
if mod(t(k), cycle_duration) < active_duration  
    adcs_power = 4.5 + rand(); % Peak power during maneuvers
else
    adcs_power = 1.5 + 0.5 * rand(); % Baseline power
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
         active_thermal_power = 2.5; % Lowered thermal load in eclipse
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
subplot (3,1,1)
plot(t/60, P_consumed, 'r', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Consumption');
grid on;

% Display stats neatly in the upper-right corner
max_body = max(P_consumed);
min_body = min(P_consumed);
avg_body = mean(P_consumed);
annotation('textbox', [0.75, 0.85, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_body, min_body, avg_body), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');


subplot (3,1,2)
plot(t/60, P_total, 'b', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Generation');
grid on;

% Display stats neatly in the upper-right corner
max_deploy_7 = max(P_total);
min_deploy_7 = min(P_total);
avg_deploy_7 = mean(P_total);
annotation('textbox', [0.75, 0.55, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_deploy_7, min_deploy_7, avg_deploy_7), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');

subplot(3,1,3) % Battery State of Charge
plot(t/60, battery_energy, 'g', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Battery SoC (%)');
title('Battery State of Charge');
grid on;

% Display stats neatly in the upper-right corner
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
title('Power Generation, Consumption, and Battery Energy of SeaHawk-1');
legend('Power Consumed', 'Power Generated');
grid on;

yyaxis right
plot(t/60, battery_energy, 'g', 'LineWidth', 1.5);
ylabel('Battery Energy (Wh)');
legend('Power Consumed', 'Power Generated', 'Battery Energy');
hold off;

% --- Display Results ---
disp(['Max Power Generated: ', num2str(max(P_total)), ' W']);
disp(['Min Power Generated: ', num2str(min(P_total)), ' W']);
disp(['Ave Power Generated: ', num2str(mean(P_total)), ' W']);