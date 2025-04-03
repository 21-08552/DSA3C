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

% --- Sun Vector (Earth-Centered Inertial) ---
i = 98.6; % Inclination angle (sun-synchronous)
dec = 23.5; % Sun declination angle (Earth axial tilt)

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

figure;
subplot(3,1,1);
plot(t/60, P_gen_deploy_7, 'r', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Generation of Deployable Panels (DSA7)');
grid on;

% Display stats neatly in the upper-right corner
max_body = max(P_gen_deploy_7);
min_body = min(P_gen_deploy_7);
avg_body = mean(P_gen_deploy_7);
annotation('textbox', [0.75, 0.85, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_body, min_body, avg_body), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');


subplot(3,1,2);
plot(t/60, P_gen_deploy_4, 'g', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Generation of Deployable Panels (DSA4)');
grid on;

% Display stats neatly in the upper-right corner
max_deploy_7 = max(P_gen_deploy_4);
min_deploy_7 = min(P_gen_deploy_4);
avg_deploy_7 = mean(P_gen_deploy_4);
annotation('textbox', [0.75, 0.55, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_deploy_7, min_deploy_7, avg_deploy_7), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');

subplot(3,1,3);
plot(t/60, P_gen_deploy_2, 'b', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Power (W)');
title('Power Generation of Deployable Panels (DSA2)');
grid on;

% Display stats neatly in the upper-right corner
max_deploy_4 = max(P_gen_deploy_2);
min_deploy_4 = min(P_gen_deploy_2);
avg_deploy_4 = mean(P_gen_deploy_2);
annotation('textbox', [0.75, 0.25, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f \nMin: %.2f \nAve: %.2f ', max_deploy_4, min_deploy_4, avg_deploy_4), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');

figure;
plot(t/60, P_total, 'k', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('Total Power Generation (W)');
title('Total Power Generation of DSA3C');
grid on;

% Display stats neatly in the upper-right corner
max_body = max(P_total);
min_body = min(P_total);
avg_body = mean(P_total);
annotation('textbox', [0.75, 0.85, 0.1, 0.1], 'String', ...
    sprintf('Max: %.2f W\nMin: %.2f W\nAve: %.2f W', max_body, min_body, avg_body), ...
    'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'white');


disp(['Max Power Generated: ', num2str(max(P_total)), ' W']);
disp(['Min Power Generated: ', num2str(min(P_total)), ' W']);
disp(['Min Power Generated: ', num2str(mean(P_total)), ' W']);
disp(R_attitude);