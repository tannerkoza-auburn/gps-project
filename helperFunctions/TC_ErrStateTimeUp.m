function [X, P] = constVelTU(oldStates, Pin, configs, dt)
%{
    Tightly-couple IMU error state update (ECEF frame) based on Groves
    txtbook.

    State vector of the form in 14.39 (with clk terms added):
    [del_Psi del_Vel del_Pos aBias gBias clkBias clkDrift]
%}


% extract inputs
ECEFpos = oldStates.ECEF;
lat = oldStates.lat; % degrees
lon = oldStates.lon; % degrees
C_b2e = oldStates.C_b2e;
clkBias = oldStates.clkBias;
clkDrift = oldStates.clkDrift;


% --- constants
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity
Omega_ie = Skew([0; 0; omega_ie]);

%--- intermediate calculations
r_es_E = R_0 / sqrt(1 - (e * sind(lat))^2) *...
    sqrt(cosd(lat)^2 + (1 - e^2)^2 * sind(lat)^2); % geocentric radius (2.137)

% import configs which determine Q matrix:
acelNoisePSD = configs.accNoise_PSD;
gyroNoisePSD = configs.gyroNoise_PSD;
accelBiasPSD = configs.accBias_PSD;
gyroBiasPSD = configs.gyroBias_PSD;
clockPhasePSD = configs.clkPhase_PSD;
clockFreqPSD = configs.clkFreq_PSD;

%___________________________ update state estimates
X = [zeros(15,1); clkBias + clkDrift*dt; clkDrift]; % closed-loop... zeroing out errors after each update


%___________________________ construct Phi (discretized) (14.50)
Phi = eye(17);
Phi(1:3,1:3) = Phi(1:3,1:3) - Omega_ie * dt;
Phi(1:3,13:15) = C_b2e * dt;
Phi(4:6,1:3) = -dt * Skew(C_b2e * meas_f_ib_b);
Phi(4:6,4:6) = Phi(4:6,4:6) - 2 * Omega_ie * dt;
Phi(4:6,7:9) = -dt * 2 * GravModel_ECEF(ECEFpos) /...
    r_es_E * ECEFpos' / sqrt (ECEFpos' *...
    ECEFpos);
Phi(4:6,10:12) = C_b2e * dt;
Phi(7:9,4:6) = eye(3) * dt;
Phi(16,17) = dt;

%______________________________ Q matrix(14.82)
Q = zeros(17);
Q(1:3,1:3) = eye(3) * gyroNoisePSD * dt;
Q(4:6,4:6) = eye(3) * acelNoisePSD * dt;
Q(10:12,10:12) = eye(3) * accelBiasPSD * dt;
Q(13:15,13:15) = eye(3) * gyroBiasPSD * dt;
Q(16,16) = clockPhasePSD * dt;
Q(17,17) = clockFreqPSD * dt;

% __________________ Update covariance
P = Phi*(Pin + .5*Q)*Phi.' + .5*Q;

end

