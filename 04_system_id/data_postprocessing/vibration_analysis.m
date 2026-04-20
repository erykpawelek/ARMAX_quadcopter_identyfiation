clc 
clear all 

%% Vibration analysis during hover 
vib_hover_filt_20 = load('vib_hover_filt_20.mat');
vib_hover_filt_20.markers.time(3) = vib_hover_filt_20.markers.time(3) + 0.94;
vib_hover_filt_160 = load('vib_hover_filt_160.mat');
vib_hover_filt_160.markers.time(3) = vib_hover_filt_160.markers.time(3) + 0.94;

figure("Name",'Time plots of Accelerations during hovering with payload dropp (LPF cutoff 20Hz');
subplot(2,1,1);
plot(vib_hover_filt_20.imu.time, vib_hover_filt_20.imu.accX, '-r', vib_hover_filt_20.imu.time, vib_hover_filt_20.imu.accY, '--b'); 
xlim([40 130])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]')
grid on;
% Loop to plot vertical lines for markers
for i = 1:length(vib_hover_filt_20.markers.time)
    m_time = vib_hover_filt_20.markers.time(i);
    m_text = string(vib_hover_filt_20.markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration X', 'Acceleration Y');
subplot(2,1,2);
plot(vib_hover_filt_20.imu.time, vib_hover_filt_20.imu.accZ, '-g'); 
xlim([40 130])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]')
grid on;

for i = 1:length(vib_hover_filt_20.markers.time)
    m_time = vib_hover_filt_20.markers.time(i);
    m_text = string(vib_hover_filt_20.markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration Z');
sgtitle('Time plots of Accelerations during hovering with payload dropp (LPF cutoff 20Hz');

figure("Name",'Time plots of Accelerations during hovering with payload dropp (LPF cutoff 160Hz');
subplot(2,1,1);
plot(vib_hover_filt_160.imu.time, vib_hover_filt_160.imu.accX, '-r', vib_hover_filt_160.imu.time, vib_hover_filt_160.imu.accY, '--b'); 
xlim([40 130])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]')
grid on;
% Loop to plot vertical lines for markers
for i = 1:length(vib_hover_filt_160.markers.time)
    m_time = vib_hover_filt_160.markers.time(i);
    m_text = string(vib_hover_filt_160.markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration X', 'Acceleration Y');
subplot(2,1,2);
plot(vib_hover_filt_160.imu.time, vib_hover_filt_160.imu.accZ, '-g'); 
xlim([40 130])
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]')
grid on;

for i = 1:length(vib_hover_filt_160.markers.time)
    m_time = vib_hover_filt_160.markers.time(i);
    m_text = string(vib_hover_filt_160.markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration Z');
sgtitle('Time plots of Accelerations during hovering with payload dropp (LPF cutoff 160Hz');

%% FFT Analysis for Hover (LPF 20Hz)

% Extracting marker strings for 20Hz data
marker_str_20 = string(vib_hover_filt_20.markers.text);

% Finding indices of specific markers
idx_hp_start_20 = find(contains(marker_str_20, 'HOVER_PAYLOAD_START'));
idx_hp_end_20   = find(contains(marker_str_20, 'HOVER_PAYLOAD_STOP'));
idx_hnp_start_20 = find(contains(marker_str_20, 'HOVER_NO_PAYLOAD_START'));
idx_hnp_end_20   = find(contains(marker_str_20, 'HOVER_NO_PAYLOAD_STOP'));

% Extracting exact time values for markers
t_hp_start_20 = vib_hover_filt_20.markers.time(idx_hp_start_20);
t_hp_end_20   = vib_hover_filt_20.markers.time(idx_hp_end_20);
t_hnp_start_20 = vib_hover_filt_20.markers.time(idx_hnp_start_20);
t_hnp_end_20   = vib_hover_filt_20.markers.time(idx_hnp_end_20);

% Creating logical masks for time segments
mask_hp_20 = (vib_hover_filt_20.imu.time >= t_hp_start_20) & (vib_hover_filt_20.imu.time <= t_hp_end_20);
mask_hnp_20 = (vib_hover_filt_20.imu.time >= t_hnp_start_20) & (vib_hover_filt_20.imu.time <= t_hnp_end_20);

% Extracting time and acceleration segments
time_hp_20 = vib_hover_filt_20.imu.time(mask_hp_20);
accX_hp_20 = vib_hover_filt_20.imu.accX(mask_hp_20);
accY_hp_20 = vib_hover_filt_20.imu.accY(mask_hp_20);

time_hnp_20 = vib_hover_filt_20.imu.time(mask_hnp_20);
accX_hnp_20 = vib_hover_filt_20.imu.accX(mask_hnp_20);
accY_hnp_20 = vib_hover_filt_20.imu.accY(mask_hnp_20);

% Calculating real sampling frequency
Fs_hp_20 = length(accX_hp_20) / (time_hp_20(end) - time_hp_20(1));
Fs_hnp_20 = length(accX_hnp_20) / (time_hnp_20(end) - time_hnp_20(1));

% Detrending data
accX_hp_20_detrend = accX_hp_20 - mean(accX_hp_20);
accY_hp_20_detrend = accY_hp_20 - mean(accY_hp_20);
accX_hnp_20_detrend = accX_hnp_20 - mean(accX_hnp_20);
accY_hnp_20_detrend = accY_hnp_20 - mean(accY_hnp_20);

% Length of signals
L_hp_20 = length(accX_hp_20_detrend);
L_hnp_20 = length(accX_hnp_20_detrend);

% FFT for X Axis (20Hz)
Y_x_hp_20 = fft(accX_hp_20_detrend);
P2_x_hp_20 = abs(Y_x_hp_20 / L_hp_20); 
P1_x_hp_20 = P2_x_hp_20(1:floor(L_hp_20/2)+1);
P1_x_hp_20(2:end-1) = 2 * P1_x_hp_20(2:end-1);

Y_x_hnp_20 = fft(accX_hnp_20_detrend);
P2_x_hnp_20 = abs(Y_x_hnp_20 / L_hnp_20);
P1_x_hnp_20 = P2_x_hnp_20(1:floor(L_hnp_20/2)+1);
P1_x_hnp_20(2:end-1) = 2 * P1_x_hnp_20(2:end-1);

% FFT for Y Axis (20Hz)
Y_y_hp_20 = fft(accY_hp_20_detrend);
P2_y_hp_20 = abs(Y_y_hp_20 / L_hp_20); 
P1_y_hp_20 = P2_y_hp_20(1:floor(L_hp_20/2)+1);
P1_y_hp_20(2:end-1) = 2 * P1_y_hp_20(2:end-1);

Y_y_hnp_20 = fft(accY_hnp_20_detrend);
P2_y_hnp_20 = abs(Y_y_hnp_20 / L_hnp_20);
P1_y_hnp_20 = P2_y_hnp_20(1:floor(L_hnp_20/2)+1);
P1_y_hnp_20(2:end-1) = 2 * P1_y_hnp_20(2:end-1);

% Frequency vectors
f_hp_20 = Fs_hp_20 * (0:(floor(L_hp_20/2))) / L_hp_20;
f_hnp_20 = Fs_hnp_20 * (0:(floor(L_hnp_20/2))) / L_hnp_20;

% Plotting FFT 20Hz - X Axis
figure('Name','FFT 20Hz - X Axis during hover'); 
subplot(2,1,1);
plot(f_hp_20, P1_x_hp_20, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Attached');
subplot(2,1,2);
plot(f_hnp_20, P1_x_hnp_20, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Detached');
sgtitle('Amplitude Spectrum - X Axis Vibrations (Hover, LPF 20Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');

% Plotting FFT 20Hz - Y Axis
figure('Name','FFT 20Hz - Y Axis during hover'); 
subplot(2,1,1);
plot(f_hp_20, P1_y_hp_20, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Attached');
subplot(2,1,2);
plot(f_hnp_20, P1_y_hnp_20, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Detached');
sgtitle('Amplitude Spectrum - Y Axis Vibrations (Hover, LPF 20Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');

%% FFT Analysis for Hover (LPF 160Hz)

% Extracting marker strings for 160Hz data
marker_str_160 = string(vib_hover_filt_160.markers.text);

% Finding indices of specific markers
idx_hp_start_160 = find(contains(marker_str_160, 'HOVER_PAYLOAD_START'));
idx_hp_end_160   = find(contains(marker_str_160, 'HOVER_PAYLOAD_STOP'));
idx_hnp_start_160 = find(contains(marker_str_160, 'HOVER_NO_PAYLOAD_START'));
idx_hnp_end_160   = find(contains(marker_str_160, 'HOVER_NO_PAYLOAD_STOP'));

% Extracting exact time values for markers
t_hp_start_160 = vib_hover_filt_160.markers.time(idx_hp_start_160);
t_hp_end_160   = vib_hover_filt_160.markers.time(idx_hp_end_160);
t_hnp_start_160 = vib_hover_filt_160.markers.time(idx_hnp_start_160);
t_hnp_end_160   = vib_hover_filt_160.markers.time(idx_hnp_end_160);

% Creating logical masks for time segments
mask_hp_160 = (vib_hover_filt_160.imu.time >= t_hp_start_160) & (vib_hover_filt_160.imu.time <= t_hp_end_160);
mask_hnp_160 = (vib_hover_filt_160.imu.time >= t_hnp_start_160) & (vib_hover_filt_160.imu.time <= t_hnp_end_160);

% Extracting time and acceleration segments
time_hp_160 = vib_hover_filt_160.imu.time(mask_hp_160);
accX_hp_160 = vib_hover_filt_160.imu.accX(mask_hp_160);
accY_hp_160 = vib_hover_filt_160.imu.accY(mask_hp_160);

time_hnp_160 = vib_hover_filt_160.imu.time(mask_hnp_160);
accX_hnp_160 = vib_hover_filt_160.imu.accX(mask_hnp_160);
accY_hnp_160 = vib_hover_filt_160.imu.accY(mask_hnp_160);

% Calculating real sampling frequency
Fs_hp_160 = length(accX_hp_160) / (time_hp_160(end) - time_hp_160(1));
Fs_hnp_160 = length(accX_hnp_160) / (time_hnp_160(end) - time_hnp_160(1));

% Detrending data (removing mean/DC offset)
accX_hp_160_detrend = accX_hp_160 - mean(accX_hp_160);
accY_hp_160_detrend = accY_hp_160 - mean(accY_hp_160);
accX_hnp_160_detrend = accX_hnp_160 - mean(accX_hnp_160);
accY_hnp_160_detrend = accY_hnp_160 - mean(accY_hnp_160);

% Length of signals
L_hp_160 = length(accX_hp_160_detrend);
L_hnp_160 = length(accX_hnp_160_detrend);

% FFT for X Axis (160Hz)
Y_x_hp_160 = fft(accX_hp_160_detrend);
P2_x_hp_160 = abs(Y_x_hp_160 / L_hp_160); 
P1_x_hp_160 = P2_x_hp_160(1:floor(L_hp_160/2)+1);
P1_x_hp_160(2:end-1) = 2 * P1_x_hp_160(2:end-1);

Y_x_hnp_160 = fft(accX_hnp_160_detrend);
P2_x_hnp_160 = abs(Y_x_hnp_160 / L_hnp_160);
P1_x_hnp_160 = P2_x_hnp_160(1:floor(L_hnp_160/2)+1);
P1_x_hnp_160(2:end-1) = 2 * P1_x_hnp_160(2:end-1);

% FFT for Y Axis (160Hz)
Y_y_hp_160 = fft(accY_hp_160_detrend);
P2_y_hp_160 = abs(Y_y_hp_160 / L_hp_160); 
P1_y_hp_160 = P2_y_hp_160(1:floor(L_hp_160/2)+1);
P1_y_hp_160(2:end-1) = 2 * P1_y_hp_160(2:end-1);

Y_y_hnp_160 = fft(accY_hnp_160_detrend);
P2_y_hnp_160 = abs(Y_y_hnp_160 / L_hnp_160);
P1_y_hnp_160 = P2_y_hnp_160(1:floor(L_hnp_160/2)+1);
P1_y_hnp_160(2:end-1) = 2 * P1_y_hnp_160(2:end-1);

% Frequency vectors
f_hp_160 = Fs_hp_160 * (0:(floor(L_hp_160/2))) / L_hp_160;
f_hnp_160 = Fs_hnp_160 * (0:(floor(L_hnp_160/2))) / L_hnp_160;

% Plotting FFT 160Hz - X Axis
figure('Name','FFT 160Hz - X Axis during hover'); 
subplot(2,1,1);
plot(f_hp_160, P1_x_hp_160, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Attached');
subplot(2,1,2);
plot(f_hnp_160, P1_x_hnp_160, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Detached');
sgtitle('Amplitude Spectrum - X Axis Vibrations (Hover, LPF 160Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');

% Plotting FFT 160Hz - Y Axis
figure('Name','FFT 160Hz - Y Axis during hover'); 
subplot(2,1,1);
plot(f_hp_160, P1_y_hp_160, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Attached');
subplot(2,1,2);
plot(f_hnp_160, P1_y_hnp_160, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Detached');
sgtitle('Amplitude Spectrum - Y Axis Vibrations (Hover, LPF 160Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');

%% FFT Analysis for Hover (Linspace Time Vector Correction - LPF 20Hz & 160Hz)

% SECTION 1: Corrected Time Vector and FFT for 20Hz

% Create a perfectly uniform time vector using linspace (No data loss)
t_uni_hp_20 = linspace(time_hp_20(1), time_hp_20(end), length(accX_hp_20));
t_uni_hnp_20 = linspace(time_hnp_20(1), time_hnp_20(end), length(accX_hnp_20));

% 2. Calculate the exact average sampling frequency based on the new uniform vector
Fs_ideal_hp_20 = 1 / (t_uni_hp_20(2) - t_uni_hp_20(1));
Fs_ideal_hnp_20 = 1 / (t_uni_hnp_20(2) - t_uni_hnp_20(1));

% 3. Detrend the RAW, untouched acceleration data (Removing DC offset)
accX_hp_20_detrend = accX_hp_20 - mean(accX_hp_20);
accY_hp_20_detrend = accY_hp_20 - mean(accY_hp_20);
accX_hnp_20_detrend = accX_hnp_20 - mean(accX_hnp_20);
accY_hnp_20_detrend = accY_hnp_20 - mean(accY_hnp_20);

% 4. Determine the length of the signals
L_hp_20 = length(accX_hp_20_detrend);
L_hnp_20 = length(accX_hnp_20_detrend);

% 5. FFT for X Axis (20Hz)
Y_x_hp_20 = fft(accX_hp_20_detrend);
P2_x_hp_20 = abs(Y_x_hp_20 / L_hp_20); 
P1_x_hp_20 = P2_x_hp_20(1:floor(L_hp_20/2)+1);
P1_x_hp_20(2:end-1) = 2 * P1_x_hp_20(2:end-1);

Y_x_hnp_20 = fft(accX_hnp_20_detrend);
P2_x_hnp_20 = abs(Y_x_hnp_20 / L_hnp_20);
P1_x_hnp_20 = P2_x_hnp_20(1:floor(L_hnp_20/2)+1);
P1_x_hnp_20(2:end-1) = 2 * P1_x_hnp_20(2:end-1);

% 6. Create proper frequency vectors based on the ideal sampling rate
f_hp_20 = Fs_ideal_hp_20 * (0:(floor(L_hp_20/2))) / L_hp_20;
f_hnp_20 = Fs_ideal_hnp_20 * (0:(floor(L_hnp_20/2))) / L_hnp_20;

% 7. Plotting FFT 20Hz - X Axis
figure('Name','FFT 20Hz - X Axis (Linspace Corrected)'); 
subplot(2,1,1);
plot(f_hp_20, P1_x_hp_20, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Attached');
subplot(2,1,2);
plot(f_hnp_20, P1_x_hnp_20, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Detached');
sgtitle('Amplitude Spectrum - X Axis Vibrations (Linspace Corrected, LPF 20Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');

% SECTION 2: Corrected Time Vector and FFT for 160Hz


% 1. Create a perfectly uniform time vector using linspace (No data loss)
t_uni_hp_160 = linspace(time_hp_160(1), time_hp_160(end), length(accX_hp_160));
t_uni_hnp_160 = linspace(time_hnp_160(1), time_hnp_160(end), length(accX_hnp_160));

% 2. Calculate the exact average sampling frequency based on the new uniform vector
Fs_ideal_hp_160 = 1 / (t_uni_hp_160(2) - t_uni_hp_160(1));
Fs_ideal_hnp_160 = 1 / (t_uni_hnp_160(2) - t_uni_hnp_160(1));

% 3. Detrend the RAW, untouched acceleration data
accX_hp_160_detrend = accX_hp_160 - mean(accX_hp_160);
accY_hp_160_detrend = accY_hp_160 - mean(accY_hp_160);
accX_hnp_160_detrend = accX_hnp_160 - mean(accX_hnp_160);
accY_hnp_160_detrend = accY_hnp_160 - mean(accY_hnp_160);

% 4. Determine the length of the signals
L_hp_160 = length(accX_hp_160_detrend);
L_hnp_160 = length(accX_hnp_160_detrend);

% 5. FFT for X Axis (160Hz)
Y_x_hp_160 = fft(accX_hp_160_detrend);
P2_x_hp_160 = abs(Y_x_hp_160 / L_hp_160); 
P1_x_hp_160 = P2_x_hp_160(1:floor(L_hp_160/2)+1);
P1_x_hp_160(2:end-1) = 2 * P1_x_hp_160(2:end-1);

Y_x_hnp_160 = fft(accX_hnp_160_detrend);
P2_x_hnp_160 = abs(Y_x_hnp_160 / L_hnp_160);
P1_x_hnp_160 = P2_x_hnp_160(1:floor(L_hnp_160/2)+1);
P1_x_hnp_160(2:end-1) = 2 * P1_x_hnp_160(2:end-1);

% 6. Create proper frequency vectors based on the ideal sampling rate
f_hp_160 = Fs_ideal_hp_160 * (0:(floor(L_hp_160/2))) / L_hp_160;
f_hnp_160 = Fs_ideal_hnp_160 * (0:(floor(L_hnp_160/2))) / L_hnp_160;

% 7. Plotting FFT 160Hz - X Axis
figure('Name','FFT 160Hz - X Axis (Linspace Corrected)'); 
subplot(2,1,1);
plot(f_hp_160, P1_x_hp_160, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Attached');
subplot(2,1,2);
plot(f_hnp_160, P1_x_hnp_160, '-r'); grid on; xlim([0 400]);
legend('Hover Payload Detached');
sgtitle('Amplitude Spectrum - X Axis Vibrations (Linspace Corrected, LPF 160Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');