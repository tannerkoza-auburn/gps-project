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
lla1(i,:) = ecef2lla(est1.pos');


% init at center of test track for testing:
lla = [32.604867, -85.487688, 700]; % deg, deg, m
ECEF = lla2ecef(lla);

% init state vector
pos(:,1) = ECEF';
vel(:,1) = [0; 0; 0];
clkBias(1) = 0;
clkDrift(1) = 0;
X(:,1) = [pos; vel; clkBias(1); clkDrift(1)];


%% TUNING
P(:,:,1) = eye(8,8);
Q = 10*eye(8,8);



%% Main Loop

% loop through all measurements
for i = 1:length(RCVR1)
    
    SVpsr = RCVR1{i,1}.L1.psr;
    psrSigma = ones(length(SVpsr),1); % PLACE HOLDER
    SVdopp = RCVR1{i,1}.L1.dopp;
    doppSigma = ones(length(SVdopp),1); % PLACE HOLDER
    SVpos = RCVR1{i,1}.L1.svPos;
    SVvel = RCVR1{i,1}.L1.svVel;
    
    % find time step between gps measurements
    dt = RCVR1{i,1}.L1.gpsTime - RCVR1{max([i-1,1]),1}.L1.gpsTime; 
        
    %%%%%%%%%%%%%%%%%%%%%%% Meas Update %%%%%%%%%%%%%%%%%%%%%%%%
    [dZ, H, R] = TC_MeasModel(X(:,i), SVpsr, psrSigma, SVdopp, doppSigma, SVpos, SVvel);
    [X(:,i), P(:,:,i)] = EKF_MeasUpdate(X(:,i), P(:,:,i), dZ, H, R);
    
    %%%%%%%%%%%%%%%%%%%%%%% Time Update %%%%%%%%%%%%%%%%%%%%%%%%%
    [X(:,i+1), P(:,:,i+1)] = constVelTU(X(:,i), P(:,:,i), Q, dt);
    
end 


%% Plotting 

% --- convert to NED
wgs84 = wgs84Ellipsoid('meter');
[N, E, D] = ecef2ned(X(1,:), X(2,:), X(3,:), lla(1), lla(2), lla(3), wgs84);



% trajectory plot
lla = ecef2lla(X(1:3,:)');
figure()
geoplot(lla(:,1), lla(:,2), '*');
% geolimits([32.5948   32.5961], [-85.2962  -85.2943]);
geobasemap satellite







