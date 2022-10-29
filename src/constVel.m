%{
    Constant velocity tightly-coupled EKF
%}

clear; clc; close all;


%% !!!!!! Testing with old lab data
load('RCVR1.mat'); % novatel on car

%% INIT
rcvr1 = gnssReceiver();
psr1 = RCVR1{1}.L1.psr;
dopp1 = RCVR1{1}.L1.dopp;
svPos1 = RCVR1{1}.L1.svPos;
svVel1 = RCVR1{1}.L1.svVel;
clkCorr1 = RCVR1{1}.L1.clkCorr;
est1 = rcvr1.pv3D(psr1,dopp1,svPos1,svVel1,clkCorr1, 1);
lla = ecef2lla(est1.pos'); % init position

% init state vector
pos(:,1) = est1.pos';
vel(:,1) = est1.vel;
clkBias(1) = est1.clock_bias;
clkDrift(1) = est1.clock_drfit;
X(:,1) = [pos; vel; clkBias(1); clkDrift(1)];


%% TUNING
P(:,:,1) = est1.P;
Q = 100*eye(8,8);



%% Main Loop
start = RCVR1{1,1}.L1.gpsTime;
stop = RCVR1{end,1}.L1.gpsTime;
time = 0:1:stop-start;
% loop through all measurements
C = physconst('LightSpeed');
for i = 1:length(RCVR1)
    
    SVpsr = RCVR1{i,1}.L1.psr;
    psrSigma = ones(length(SVpsr),1); % PLACE HOLDER
    SVdopp = RCVR1{i,1}.L1.dopp;
    doppSigma = 10000000*ones(length(SVdopp),1); % PLACE HOLDER
    SVpos = RCVR1{i,1}.L1.svPos;
    SVvel = RCVR1{i,1}.L1.svVel;
    clkCorr = RCVR1{i}.L1.clkCorr;
    
    % correct pseudoranges ( may need to be subtracted )
    SVpsr = SVpsr + clkCorr*C;
    
    % find time step between gps measurements
    dt = RCVR1{i,1}.L1.gpsTime - RCVR1{max([i-1,1]),1}.L1.gpsTime; 
        
    %%%%%%%%%%%%%%%%%%%%%%% Meas Update %%%%%%%%%%%%%%%%%%%%%%%%
    if i ~= 0
        [dZ, H, R] = TC_MeasModel(X(:,i), SVpsr, psrSigma, SVdopp, doppSigma, SVpos, SVvel);
        [X(:,i), P(:,:,i), dX(:,i)] = EKF_MeasUpdate(X(:,i), P(:,:,i), dZ, H, R);
    end 
    
    
    %%%%%%%%%%%%%%%%%%%%%%% Time Update %%%%%%%%%%%%%%%%%%%%%%%%%
    if i ~= length(RCVR1)
        [X(:,i+1), P(:,:,i+1)] = constVelTU(X(:,i), P(:,:,i), Q, dt);
    end 
end 


%% Plotting 

% --- convert to NED
wgs84 = wgs84Ellipsoid('meter');
[N, E, D] = ecef2ned(X(1,:), X(2,:), X(3,:), lla(1), lla(2), lla(3), wgs84);



% trajectory plot
lla = ecef2lla(X(1:3,:)');
figure()
geoplot(lla(:,1), lla(:,2));
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



