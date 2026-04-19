clc 
clear all 

load('z_axis_ident.mat')
markers.time(4) = markers.time(4) + 0.94;

%% Accelerations plots and PWM plots
figure('Name','Acceleration values along X,Y,Z axis');
subplot(2,1,1);
plot(imu.time, imu.accX, '-r', imu.time, imu.accY, '--b'); xlim([50 120])
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
grid on;
for i = 1:length(markers.time)
    m_time = markers.time(i);
    m_text = string(markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration X', 'Acceleration Y');
subplot(2,1,2);
plot(imu.time, imu.accZ, '-g'); xlim([50 120])
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
grid on;
for i = 1:length(markers.time)
    m_time = markers.time(i);
    m_text = string(markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration Z');
sgtitle('Accelaration plots during whole identyfication sequence');

figure('Name', 'Motor PWM Signals during whole identification sequence');
u_mean_raw_plot = (double(rcou.c1) + double(rcou.c2) + double(rcou.c3) + double(rcou.c4)) / 4;
plot(rcou.time, u_mean_raw_plot, 'g', 'LineWidth', 1.2); 
xlim([50 120]); 
xlabel('Time [s]'); 
ylabel('Average PWM [\mus]');
title('Average Motor PWM Signal with Event Markers');
grid on;
hold on;
for i = 1:length(markers.time)
    m_time = markers.time(i);
    xline(m_time, '--m', 'LineWidth', 1.5);   
end
legend('Average Motor PWM');

%% System identification - Data Preparation & ARMAX Formatting

% Extracting marker times
marker_str = string(markers.text);
idx_noise1_start = find(contains(marker_str, 'NOISE_1_START'));
idx_noise1_end   = find(contains(marker_str, 'NOISE_1_STOP'));
idx_noise2_start = find(contains(marker_str, 'NOISE_2_START'));
idx_noise2_end   = find(contains(marker_str, 'NOISE_2_STOP'));

t_noise1_start = markers.time(idx_noise1_start);
t_noise1_end   = markers.time(idx_noise1_end);
t_noise2_start = markers.time(idx_noise2_start);
t_noise2_end   = markers.time(idx_noise2_end);

% Mathematical time grid for ARMAX Ts = 0.02s
Fs_armax = 50;
Ts = 1 / Fs_armax;
t_grid_noise1 = t_noise1_start : Ts : t_noise1_end;
t_grid_noise2 = t_noise2_start : Ts : t_noise2_end;

% PROCESSING NOISE 1 (Payload Attached) 
% Extract and interpolate output: Altitude
y_alt_n1 = interp1(ctun.time, ctun.alt, t_grid_noise1, 'linear')';
y_alt_n1_detrend = y_alt_n1 - mean(y_alt_n1);

% Extract, average, and interpolate input: Motor PWM
u_mean_raw = (double(rcou.c1) + double(rcou.c2) + double(rcou.c3) + double(rcou.c4)) / 4;
u_mean_n1 = interp1(rcou.time, u_mean_raw, t_grid_noise1, 'linear')';

% Drift Compensation
roll_rad_n1 = deg2rad(interp1(att.time, att.roll, t_grid_noise1, 'linear')');
pitch_rad_n1 = deg2rad(interp1(att.time, att.pitch, t_grid_noise1, 'linear')');
u_eff_n1 = u_mean_n1 .* cos(roll_rad_n1) .* cos(pitch_rad_n1);
u_eff_n1_detrend = u_eff_n1 - mean(u_eff_n1);

data_noise1 = iddata(y_alt_n1_detrend, u_eff_n1_detrend, Ts, ...
    'InputName', 'Effective PWM', 'OutputName', 'Altitude');

% PROCESSING NOISE 2 (Payload Detached)
% Extract and interpolate output: Altitude
y_alt_n2 = interp1(ctun.time, ctun.alt, t_grid_noise2, 'linear')';
y_alt_n2_detrend = y_alt_n2 - mean(y_alt_n2);

% Extract and interpolate input: Motor PWM
u_mean_n2 = interp1(rcou.time, u_mean_raw, t_grid_noise2, 'linear')';

% Drift Compensation 
roll_rad_n2 = deg2rad(interp1(att.time, att.roll, t_grid_noise2, 'linear')');
pitch_rad_n2 = deg2rad(interp1(att.time, att.pitch, t_grid_noise2, 'linear')');
u_eff_n2 = u_mean_n2 .* cos(roll_rad_n2) .* cos(pitch_rad_n2);
u_eff_n2_detrend = u_eff_n2 - mean(u_eff_n2);

% Create iddata object
data_noise2 = iddata(y_alt_n2_detrend, u_eff_n2_detrend, Ts, ...
    'InputName', 'Effective PWM', 'OutputName', 'Altitude');

% PLOTTING PREPARED DATA
figure('Name', 'Prepared Data Overview');
subplot(2,1,1);
plot(data_noise1);
title('Prepared ARMAX Data - Noise 1 (Payload Attached)');
subplot(2,1,2);
plot(data_noise2);
title('Prepared ARMAX Data - Noise 2 (Payload Detached)');

% VISUALIZATION: RAW VS INTERPOLATED DATA 
figure('Name', 'Interpolation Verification');
plot(t_grid_noise1, y_alt_n1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Interpolated Grid (Strict 50Hz)');hold on;
grid on;
plot(ctun.time, ctun.alt, 'g.', 'MarkerSize', 8, 'DisplayName', 'Raw Log Data (Variable Step)');
xlim([t_noise1_start, t_noise1_end]);

title('Interpolation Verification - Full Altitude Waveform');
xlabel('Time [s]');
ylabel('Altitude [m]');
legend('Location', 'best');

%% ARMAX MODEL ESTIMATION 1

armax_orders = [3 3 3 2];

disp('Estimating ARMAX models... This might take a moment.');

% Estimate models using the prepared iddata objects
sys_armax_n1 = armax(data_noise1, armax_orders);
sys_armax_n2 = armax(data_noise2, armax_orders);

% Display the estimated discrete-time polynomials in the command window
disp('--- ARMAX Model: Noise 1 (Payload Attached) ---');
sys_armax_n1
disp('--- ARMAX Model: Noise 2 (Payload Detached) ---');
sys_armax_n2

% --- MODEL VALIDATION (COMPARE FIT) ---
figure('Name', 'ARMAX Model Validation');
subplot(2,1,1);
compare(data_noise1, sys_armax_n1);
title('Model Fit - Noise 1 (Payload Attached)');

subplot(2,1,2);
compare(data_noise2, sys_armax_n2);
title('Model Fit - Noise 2 (Payload Detached)');

% RESIDUAL ANALYSIS 
% Create a figure for the Payload Attached model
figure('Name', 'Residual Analysis - Noise 1 (Payload Attached)');
% The 'resid' function computes and plots the autocorrelation of residuals 
% and the cross-correlation between residuals and the input.
resid(data_noise1, sys_armax_n1);
title('Residual Analysis - Noise 1 (Payload Attached)');

% Create a figure for the Payload Detached model
figure('Name', 'Residual Analysis - Noise 2 (Payload Detached)');
resid(data_noise2, sys_armax_n2);
title('Residual Analysis - Noise 2 (Payload Detached)');

disp('Estimating Empirical Frequency Responses...');
sys_empirical_n1 = spa(data_noise1);
sys_empirical_n2 = spa(data_noise2);

% Setup Bode plot options for better readability (using Hertz instead of rad/s)
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseMatching = 'on';

% Plot 1: Payload Attached (Empirical vs ARMAX)
figure('Name', 'System Frequency Response (Payload Attached)');
% Plot empirical data in solid blue, ARMAX model in dashed red
bode(sys_empirical_n1, 'b', sys_armax_n1, 'r--', opts);
grid on;
title('Bode Plot Comparison: Noise 1 (Payload Attached)');
legend('Empirical Data (Real)', 'ARMAX Model [3 3 3 2]', 'Location', 'southwest');

% Plot 2: Payload Detached (Empirical vs ARMAX)
figure('Name', 'System Frequency Response (Payload Detached)');
bode(sys_empirical_n2, 'b', sys_armax_n2, 'r--', opts);
grid on;
title('Bode Plot Comparison: Noise 2 (Payload Detached)');
legend('Empirical Data (Real)', 'ARMAX Model [3 3 3 2]', 'Location', 'southwest');

%% Flight Simulation - Step Response
% Simulating a sudden, constant increase in PWM (e.g., pilot raises throttle)
figure('Name', 'Step Response: Altitude tracking simulation');

% We simulate a sudden +50 microseconds increase in effective PWM
opt = stepDataOptions('StepAmplitude', 50);

% Plot the step response for both models for a duration of 3 seconds
step(sys_armax_n1, 'r', sys_armax_n2, 'b', 3, opt);
grid on;

title('Simulated Flight Command: Reaction to sudden +50\mus PWM step');
xlabel('Time [s]');
ylabel('Altitude Change [m]');
legend('Noise 1 (Payload Attached)', 'Noise 2 (Payload Detached)', 'Location', 'northwest');

%% ARMAX MODEL ESTIMATION 2

armax_orders = [4 4 6 2];

disp('Estimating ARMAX models... This might take a moment.');

% Estimate models using the prepared iddata objects
sys_armax_n1 = armax(data_noise1, armax_orders);
sys_armax_n2 = armax(data_noise2, armax_orders);

% Display the estimated discrete-time polynomials in the command window
disp('--- ARMAX Model: Noise 1 (Payload Attached) ---');
sys_armax_n1
disp('--- ARMAX Model: Noise 2 (Payload Detached) ---');
sys_armax_n2

% --- MODEL VALIDATION (COMPARE FIT) ---
figure('Name', 'ARMAX Model Validation');
subplot(2,1,1);
compare(data_noise1, sys_armax_n1);
title('Model Fit - Noise 1 (Payload Attached)');

subplot(2,1,2);
compare(data_noise2, sys_armax_n2);
title('Model Fit - Noise 2 (Payload Detached)');

% RESIDUAL ANALYSIS 
% Create a figure for the Payload Attached model
figure('Name', 'Residual Analysis - Noise 1 (Payload Attached)');
% The 'resid' function computes and plots the autocorrelation of residuals 
% and the cross-correlation between residuals and the input.
resid(data_noise1, sys_armax_n1);
title('Residual Analysis - Noise 1 (Payload Attached)');

% Create a figure for the Payload Detached model
figure('Name', 'Residual Analysis - Noise 2 (Payload Detached)');
resid(data_noise2, sys_armax_n2);
title('Residual Analysis - Noise 2 (Payload Detached)');