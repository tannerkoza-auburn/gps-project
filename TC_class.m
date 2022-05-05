%{
    error-state tightly-coupled EKF

    State vector of the form in 14.39 (with clk terms added):
    [del_Psi del_Vel del_Pos aBias gBias clkBias clkDrift]
%}
clear all; close all; clc;
load('classData_CombinedMats')

% select KVH or cam IMU (default KVH)
% cam = 0;
timeSort = timeSortKVH;
% imu.IMU = kvh.IMU;
% if cam
%     timeSort = timeSortCam;
%     imu.IMU = camIMU.IMU;
%     IMUmat = camIMUmat;
% end 
imu.IMU = IMUmat;

% KVH bias est:
aBias(:,1) = [mean(IMUmat(1:50,2:3))'; 0];
aBias(:,1) = [0; 0; 0];
gBias(:,1) = mean(IMUmat(1:50,5:7))';
gBias(:,1) = [0; 0; 0];

%% Configs

% --- Setting Q
configs.accNoise_PSD = 0.2^2; % (micro-g^2 per Hz, converted to m^2 s^-3) 
configs.gyroNoise_PSD = 0.01^2; % (deg^2 per hour, converted to rad^2/s)       
configs.accBias_PSD = 1.0E-5; % bias random walk PSD (m^2 s^-5)
configs.gyroBias_PSD = 4.0E-11; % bias random walk PSD (rad^2 s^-3)
configs.clkPhase_PSD = 1; % clock phase-drift PSD (m^2/s) 
configs.clkFreq_PSD = 1; % clock frequency-drift PSD (m^2/s^3)

% --- Setting P with uncertainties
mg2ms = 9.80665E-6; % micro gs to m/s^2
att_Unc = deg2rad(2) * eye(3);   
vel_Unc = 0.1 * eye(3);      
pos_Unc = 10 * eye(3);
abias_Unc = 10000*mg2ms * eye(3); % micro-g to m/s^2
gbias_Unc = ( deg2rad(200) / 3600 ) * eye(3); % deg/hr to rad/s
clkBias_Unc = 10; % m
clkDrift_Unc = 0.1; % m/s
P(:,:,1) = blkdiag(att_Unc.^2, vel_Unc.^2, pos_Unc.^2, abias_Unc.^2, gbias_Unc.^2,...
    clkBias_Unc^2, clkDrift_Unc^2);




%% Init with initial GPS solution:
r = gnssReceiver();
for i = 1:length(svMat(:))
    gpsSol(i) = r.pv3D(svMat(i).psr, svMat(i).dopp, svMat(i).svPos,...
    svMat(i).svVel, svMat(i).clkCorr, 1);
    gpslla(i,:) = ecef2lla(gpsSol(i).pos');
end 

figure()
geoplot(gpslla(:,1), gpslla(:,2), '*')
geobasemap satellite

%----- init ECEF
% lla(1,:) = lla; % init lla
pos(:,1) = gpsSol(i).pos'; 
vel(:,1) = [0;0;0];

% init rotation matrix C_b2e
Eul(:,1) = [0; 0; -90]; % import init heading in deg
[C_n2b, C_b2n(:,:,1)] = NED_to_Body(Eul);
[C_n2e] = NED_to_ECEF(gpslla(1,1), gpslla(1,2));
C_b2e(:,:,1) = C_n2e*C_b2n;

%---- init NED
NED(:,1) = [0; 0; 0];
vNED(:,1) = [0; 0; 0];
lat(1) = gpslla(1,1);
lon(1) = gpslla(1,2);
h(1) = gpslla(1,3);

%--- init clk bias and drift, X and P
clkBias(1) = gpsSol(1).clock_bias;
clkDrift(1) = gpsSol(1).clock_drfit;
% P(:,:,1) = gpsSol(1).P;
X(:,1) = zeros(17,1);
X(16:17,1) = [clkBias(1); clkDrift(1)];


%% Main Loop

% kvh.IMU(:,1) = kvh.IMU(:,1) - kvh.IMU(1,1); % reset time

oldTime = 0;
C = physconst('LightSpeed');
gpsIdx = 1; % gps measurement index
imuIdx = 1; % imu measurement index

for i = 1:1000  %length(timeSort)
    
    % check time or measurement update
    if timeSort(i,2) == 1
       timeUpdate = 0; measUpdate = 1;
    else
       timeUpdate = 1; measUpdate = 0; 
    end
    
    % store old PVA
    estlla = ecef2lla(pos(:,i)');
    oldStates.pos = pos(:,i);
    oldStates.vel = vel(:,i);
    oldStates.lat = estlla(1);
    oldStates.lon = estlla(2);
    oldStates.C_b2e = C_b2e(:,:,i);
    oldStates.clkBias = clkBias(i);
    oldStates.clkDrift = clkDrift(i);
    
    
    if timeUpdate && i < length(timeSort)
        

        % find time step for this iteration
        dt(i) = IMUmat(imuIdx,1) - oldTime;
      
        % extract and correct meas with current bias estimate
        f_ib_b = imu.IMU(imuIdx,2:4)' - aBias(:,i);
        omega_ib_b = imu.IMU(imuIdx,5:7)' - gBias(:,i);    
    
        % mechanize imu
        [pos(:,i+1), vel(:,i+1), C_b2e(:,:,i+1)] = Mechanize_ECEF(f_ib_b,...
            omega_ib_b, dt(i), oldStates);
        
        [lat(i+1), lon(i+1), h(i+1), NED(:,i+1), vNED(:,i+1), C_b2n(:,:,i+1)] = Mechanize_NED(f_ib_b, omega_ib_b, lat(i), lon(i), h(i),...
             NED(:,i), vNED(:,i), C_b2n(:,:,i), dt(i));
        
        
        % perform time update
        [X(:,i+1), P(:,:,i+1)] = TC_ErrStateTimeUp(oldStates, P(:,:,i), f_ib_b, configs, dt(i));
        
        
        % extract updated clk biases and propagate biases
        aBias(:,i+1) = aBias(:,i);
        gBias(:,i+1) = gBias(:,i);
        clkBias(i+1) = X(16);
        clkDrift(i+1) = X(17);
        
        % update old time for the next time update
        oldTime = IMUmat(imuIdx,1);
        
        % update imu indices for next time update
        imuIdx = imuIdx + 1;
        
    end 
    
    if measUpdate
   
        % unpack satellite data
        SVpsr = svMat(gpsIdx).psr;
        psrSigma = sqrt(svMat(gpsIdx).psrVar);
        SVdopp = svMat(gpsIdx).dopp;
        doppSigma = sqrt(svMat(gpsIdx).doppVar);
        SVpos = svMat(gpsIdx).svPos;
        SVvel = svMat(gpsIdx).svVel;
        clkCorr = svMat(gpsIdx).clkCorr;
        
        % correct pseudoranges
        SVpsr = SVpsr + clkCorr*C;
        
        % perform correction
        [dZ, H, R] = TC_ErrStateMeasModel(oldStates, SVpsr, psrSigma, SVdopp,...
            doppSigma, SVpos, SVvel);
        [X(:,i), P(:,:,i), dX(:,i)] = EKF_MeasUpdate(X(:,i), P(:,:,i), dZ, H, R);
        
        % update states and biases
        pos(:,i) = pos(:,i) - X(4:6,i);
        vel(:,i) = vel(:,i) - X(7:9,i);
        C_b2e(:,:,i) =  (eye(3) - Skew(X(1:3,i))) * C_b2e(:,:,i);
        aBias(:,i) = aBias(:,i) + X(10:12,i);
        gBias(:,i) = gBias(:,i) + X(13:15,i);
        clkBias(i) = clkBias(i) + X(16);
        clkDrift(i) = clkDrift(i) + X(17);
        
        % propagate states forward since no time update performed:
        pos(:,i+1) = pos(:,i);
        vel(:,i+1) = vel(:,i);
        C_b2e(:,:,i+1) = C_b2e(:,:,i);
        clkBias(i+1) = clkBias(i);
        clkDrift(i+1) = clkDrift(i);
        aBias(:,i+1) = aBias(:,i);
        gBias(:,i+1) = gBias(:,i);
        X(:,i+1) = X(:,i);
        P(:,:,i+1) = P(:,:,i);
        
        % propagate NED
        lat(i+1) = lat(i);
        lon(i+1) = lon(i);
        h(i+1) = h(i);
        NED(:,i+1) = NED(:,i);
        vNED(:,i+1) = vNED(:,i);
        C_b2n(:,:,i+1) = C_b2n(:,:,i);
        
        gpsIdx = gpsIdx + 1; % iterate for next GPS measurement index
        
    end
    
    eul = rad2deg(rotm2eul(C_b2n(:,:,i))); % extract Euler angles of misalign
    mis_est(:,i) = [eul(3); eul(2); eul(1)]; % store current misalign est
    
end 

lla_out = ecef2lla(pos');

figure()
geoplot(lla_out(:,1), lla_out(:,2))
geobasemap satellite

figure()
plot(mis_est(1,:))
title('Euler Angles')
hold on
plot(mis_est(2,:))
plot(mis_est(3,:))
legend('Roll', 'Pitch', 'Yaw')

figure()
plot(Mkz.gpsHeading(:,1), Mkz.gpsHeading(:,2))

figure()
plot(Mkz.gpsGyro(:,1), Mkz.gpsGyro(:,4), '*');
hold on 
plot(kvh.IMU(:,1), kvh.IMU(:,7));

figure()
subplot(3,1,1)
plot(NED(1,:))
title('North')
subplot(3,1,2)
plot(NED(2,:))
title('East')
subplot(3,1,3)
plot(NED(3,:))
title('Down')

figure()
subplot(3,1,1)
plot(vNED(1,:))
title('vel North')
subplot(3,1,2)
plot(vNED(2,:))
title('vel East')
subplot(3,1,3)
plot(vNED(3,:))
title('vel Down')


figure()
subplot(2,1,1)
plot(kvh.accel(:,1))
hold on 
title('Body-frame accel')
plot(kvh.accel(:,2))
plot(kvh.accel(:,3))
ylabel('m/s^2')
legend('x','y','z')
subplot(2,1,2)
plot(kvh.gyro(:,1))
hold on 
title('Body-frame gyro')
plot(kvh.gyro(:,2))
ylabel('rad/s')
plot(kvh.gyro(:,3))
legend('x','y','z')


