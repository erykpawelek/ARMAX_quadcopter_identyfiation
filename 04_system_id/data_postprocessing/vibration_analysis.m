clc 
clear all 

load('accel_filter_set_160.mat')
markers.time(4) = markers.time(4) + 0.94;

%% Accelerations plots
figure;
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

%% Vibrations analisys

%Extracting data periods
marker_str = string(markers.text);
idx_noise1_start = find(contains(marker_str, 'NOISE_1_START'));
idx_noise1_end   = find(contains(marker_str, 'NOISE_1_STOP'));
idx_noise2_start = find(contains(marker_str, 'NOISE_2_START'));
idx_noise2_end   = find(contains(marker_str, 'NOISE_2_STOP'));

t_noise1_start = markers.time(idx_noise1_start);
t_noise1_end   = markers.time(idx_noise1_end);
t_noise2_start = markers.time(idx_noise2_start);
t_noise2_end   = markers.time(idx_noise2_end);

mask_noise1 = (imu.time >= t_noise1_start) & (imu.time <= t_noise1_end);
mask_noise2 = (imu.time >= t_noise2_start) & (imu.time <= t_noise2_end);

time_noise1_segment = imu.time(mask_noise1);
time_noise2_segment = imu.time(mask_noise2);
accX_noise1_segment = imu.accX(mask_noise1);
accX_noise2_segment = imu.accX(mask_noise2);
accY_noise1_segment = imu.accY(mask_noise1);
accY_noise2_segment = imu.accY(mask_noise2);

figure;
subplot(2,1,1);

plot(time_noise1_segment, accX_noise1_segment, '-r');
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
legend('Acceleration X')
grid on;
subplot(2,1,2);
plot(time_noise1_segment, accY_noise1_segment, '-b');
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
legend('Acceleration Y')
grid on;
sgtitle('Extracted segment of accelerations during noise excitation with payload attached (Low pass filter cutoff freqiency set to 160Hz)');

figure;
subplot(2,1,1);
plot(time_noise2_segment, accX_noise2_segment, '-r');
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
legend('Acceleration X')
grid on;
subplot(2,1,2);
plot(time_noise2_segment, accY_noise2_segment, '-b');
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
legend('Acceleration Y')
grid on;
sgtitle('Extracted segment of accelerations during noise excitation with payload detached (Low pass filter cutoff freqiency set to 160Hz)');

Fs1_real = length(accX_noise1_segment) / (time_noise1_segment(end) - time_noise1_segment(1));
Fs2_real = length(accX_noise2_segment) / (time_noise2_segment(end) - time_noise2_segment(1));

% Displaing real sampling freq
disp(['Real Fs for Noise 1: ', num2str(Fs1_real), ' Hz']);
disp(['Real Fs for Noise 2: ', num2str(Fs2_real), ' Hz']);

% Data detrening
accX_n1_detrend = accX_noise1_segment - mean(accX_noise1_segment);
accX_n2_detrend = accX_noise2_segment - mean(accX_noise2_segment);
accY_n1_detrend = accY_noise1_segment - mean(accY_noise1_segment);
accY_n2_detrend = accY_noise2_segment - mean(accY_noise2_segment);

% Number of samples
L1 = length(accX_n1_detrend);
L2 = length(accX_n2_detrend);

% FFT X
Y1_x = fft(accX_n1_detrend);
P2_1_x = abs(Y1_x / L1); 
P1_1_x = P2_1_x(1:floor(L1/2)+1);
P1_1_x(2:end-1) = 2 * P1_1_x(2:end-1);

Y2_x = fft(accX_n2_detrend);
P2_2_x = abs(Y2_x / L2);
P1_2_x = P2_2_x(1:floor(L2/2)+1);
P1_2_x(2:end-1) = 2 * P1_2_x(2:end-1);

% FFT Y
Y1_y = fft(accY_n1_detrend);
P2_1_y = abs(Y1_y / L1); 
P1_1_y = P2_1_y(1:floor(L1/2)+1);
P1_1_y(2:end-1) = 2 * P1_1_y(2:end-1);

Y2_y = fft(accY_n2_detrend);
P2_2_y = abs(Y2_y / L2);
P1_2_y = P2_2_y(1:floor(L2/2)+1);
P1_2_y(2:end-1) = 2 * P1_2_y(2:end-1);

% Frequency axis
f1 = Fs1_real * (0:(floor(L1/2))) / L1;
f2 = Fs2_real * (0:(floor(L2/2))) / L2;

figure; 
subplot(2,1,1);
plot(f1, P1_1_x, '-r'); grid on; xlim([0 400]);
legend('Payload Attached')
subplot(2,1,2);
plot(f2, P1_2_x, '-r'); grid on; xlim([0 400]);
legend('Payload Detached')

 
sgtitle('Normalized Single-Sided Amplitude Spectrum - X Axis Vibrations (Low pass filter cutoff freqiency set to 160Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');

figure;
subplot(2,1,1);
plot(f1, P1_1_y, '-r'); grid on; xlim([0 400]);
legend('Payload Attached')
subplot(2,1,2);
plot(f2, P1_2_y, '-r'); grid on; xlim([0 400]);
legend('Payload Detached')

sgtitle('Normalized Single-Sided Amplitude Spectrum - Y Axis Vibrations (Low pass filter cutoff freqiency set to 160Hz)');
xlabel('Frequency [Hz]'); ylabel('Normalised amplitude');
