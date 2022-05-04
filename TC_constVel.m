%{
    Constant velocity tightly-coupled EKF
%}

clear; clc; close all;


%% !!!!!! Testing with old lab data
load('figure8_CombinedMats.mat'); % novatel on car
load('figure8_rtk.mat'); % rtk data;

%% INIT
% Init with initial GPS solution:
r = gnssReceiver();
svMat = svMat(:,1:end);
for i = 1:length(svMat(:))
    gpsSol(i) = r.pv3D(svMat(i).psr, svMat(i).dopp, svMat(i).svPos,...
    svMat(i).svVel, svMat(i).clkCorr, 1);
    gpslla(i,:) = ecef2lla(gpsSol(i).pos');
end 


% init state vector
pos(:,1) = gpsSol(1).pos';
pos(:,1) = data.ecef(:,1)';
vel(:,1) = gpsSol(1).vel;
clkBias(1) = gpsSol(1).clock_bias;
clkDrift(1) = gpsSol(1).clock_drfit;
X(:,1) = [pos; vel; clkBias(1); clkDrift(1)];


%% TUNING
P(:,:,1) = gpsSol(1).P;
Q = 1*eye(8,8);


%% Main Loop
start = svMat(1).time;
stop = svMat(1).time;
time = 0:1:stop-start;
% loop through all measurements
C = physconst('LightSpeed');
for i = 1:length(svMat)
    
    % unpack satellite data
    SVpsr = svMat(i).psr;
    psrSigma = sqrt(svMat(i).psrVar);
    SVdopp = svMat(i).dopp;
    doppSigma = sqrt(svMat(i).doppVar);
    SVpos = svMat(i).svPos;
    SVvel = svMat(i).svVel;
    clkCorr = svMat(i).clkCorr;    
    
    % correct pseudoranges ( may need to be subtracted )
    SVpsr = SVpsr + clkCorr*C;
    
    % find time step between gps measurements
    dt = svMat(i).time - svMat(max([i-1,1])).time; 
        
    %%%%%%%%%%%%%%%%%%%%%%% Meas Update %%%%%%%%%%%%%%%%%%%%%%%%
    if i ~= 1
        [dZ, H, R] = TC_MeasModel(X(:,i), SVpsr, psrSigma, SVdopp, doppSigma, SVpos, SVvel);
        [X(:,i), P(:,:,i), dX(:,i)] = EKF_MeasUpdate(X(:,i), P(:,:,i), dZ, H, R);
    end 
    
    
    %%%%%%%%%%%%%%%%%%%%%%% Time Update %%%%%%%%%%%%%%%%%%%%%%%%%
    if i ~= length(svMat)
        [X(:,i+1), P(:,:,i+1)] = constVelTU(X(:,i), P(:,:,i), Q, dt);
    end 
    
end 


%% Plotting 

% --- convert to NED
wgs84 = wgs84Ellipsoid('meter');
[N, E, D] = ecef2ned(X(1,:), X(2,:), X(3,:), gpslla(1), gpslla(2), gpslla(3), wgs84);



% trajectory plot
lla = ecef2lla(X(1:3,:)');
figure()
geoplot(lla(:,1), lla(:,2), 'lineWidth', 2);
hold on 
geoplot(gpslla(:,1), gpslla(:,2), 'lineWidth', 2);
hold on
geoplot(data.lla(1,:), data.lla(2,:), 'lineWidth', 2);
% geolimits([32.5948   32.5961], [-85.2962  -85.2943]);
geobasemap satellite


% NED plot
figure('Name', 'NED position Estimates')
subplot(3,1,1)
plot(time,N)
xlabel('Time (s)')
ylabel('Meters')
title('North')
subplot(3,1,2)
plot(time,E)
xlabel('Time (s)')
ylabel('Meters')
subplot(3,1,3)
plot(time,D)
xlabel('Time (s)')
ylabel('Meters')





