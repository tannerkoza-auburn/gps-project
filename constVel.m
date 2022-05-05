%{
    Constant velocity tightly-coupled EKF
%}

clear; clc; close all;


%% !!!!!! Testing with old lab data
load('RCVR1.mat'); % novatel on car
load('novatel_2022-01-26-13-24-45')
load('UWBclassData');
rtklla = data.lla;

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
%P(:,:,1) = est1.P; % [pos vel clkBias]
P(:,:,1) = eye(8);
Q = 10*eye(8,8);



%% Regular GPS
for i = 1:length(RCVR1)
    SVpsr = RCVR1{i,1}.L1.psr;
    psrSigma = 2*ones(length(SVpsr),1); % PLACE HOLDER
    SVdopp = RCVR1{i,1}.L1.dopp;
    doppSigma = 2*ones(length(SVdopp),1); % PLACE HOLDER
    SVpos = RCVR1{i,1}.L1.svPos;
    SVvel = RCVR1{i,1}.L1.svVel;
    clkCorr = RCVR1{i}.L1.clkCorr;   

    %%%%%%%%%%%%%%%%%%%%%%% Regular GPS %%%%%%%%%%%%%%%%%%%%%%%%%%%
    est(i) = rcvr1.pv3D(SVpsr, SVdopp, SVpos,SVvel, clkCorr, 1);
    gpslla(i,:) = ecef2lla(est(i).pos');
end   


%% Main Loop
start = RCVR1{1,1}.L1.gpsTime;
stop = RCVR1{end,1}.L1.gpsTime;
time = 0:1:stop-start;
% loop through all measurements
C = physconst('LightSpeed');

idx = 1; % indexing for sats
for i = 1:length(uwbRange)
    
    
    % find time step between gps measurements
    %dt = RCVR1{i,1}.L1.gpsTime - RCVR1{max([i-1,1]),1}.L1.gpsTime; 
     dt = uwbTime(i) - uwbTime(max([i-1,1]));   
    
    %%%%%%%%%%%%%%%%%%%%%%% Meas Update %%%%%%%%%%%%%%%%%%%%%%%%
    if i ~= 0

        % satellite update
        if uwbTime(i) == time(idx)
            SVpsr = RCVR1{idx,1}.L1.psr;
            psrSigma = 2*ones(length(SVpsr),1); % PLACE HOLDER
            SVdopp = RCVR1{idx,1}.L1.dopp;
            doppSigma = 2*ones(length(SVdopp),1); % PLACE HOLDER
            SVpos = RCVR1{idx,1}.L1.svPos;
            SVvel = RCVR1{idx,1}.L1.svVel;
            clkCorr = RCVR1{idx}.L1.clkCorr;
            
            % reduce # of satellites used along with the UWBs
            %nSats = length(SVpsr);
            nSats = 2;
            SVpsr = SVpsr(1:nSats);
            psrSigma = psrSigma(1:nSats);
            SVdopp = SVdopp(1:nSats);
            doppSigma = doppSigma(1:nSats);
            SVpos = SVpos(1:nSats,:);
            SVvel = SVvel(1:nSats,:);
            clkCorr = clkCorr(1:nSats);

            % correct pseudoranges for TC ( may need to be subtracted )
            SVpsr = SVpsr + clkCorr*C;
            
            % update with sats
            [dZ, H, R] = TC_MeasModel(X(:,i), SVpsr, psrSigma, SVdopp, doppSigma, SVpos, SVvel);       
            idx = idx + 1; % iterate sv idx 
            
              % correct       
        [X(:,i), P(:,:,i), dX(:,i)] = EKF_MeasUpdate(X(:,i), P(:,:,i), dZ, H, R);
        end 
        
%         % update with UWBs
%         [dZuwb, Huwb, Ruwb] = TC_MeasModelUWB(X(:,i), uwbRange(i,:), UWBsigma*ones(length(uwbRange(i,:)),1), uwbPos);   
%         
%         if uwbTime(i) == time(idx) % indicates SV and UWB measurement
%             dZ = [dZ; dZuwb];
%             H = [H; Huwb];
%             R = blkdiag(R, Ruwb);
%         else
%             dZ = dZuwb;
%             H = Huwb;
%             R = Ruwb;
%         end 
        
      
        
    end 
    
    
    %%%%%%%%%%%%%%%%%%%%%%% Time Update %%%%%%%%%%%%%%%%%%%%%%%%%
    if i ~= length(uwbRange)
        [X(:,i+1), P(:,:,i+1)] = constVelTU(X(:,i), P(:,:,i), Q, dt);
    end 
    
end 


%% Plotting 

b = '#0072BD'; % blue
o = '#D95319'; % orange
y = '#EDB120'; % yellow


% --- convert to NED
wgs84 = wgs84Ellipsoid('meter');
[N, E, D] = ecef2ned(X(1,:), X(2,:), X(3,:), lla(1), lla(2), lla(3), wgs84);



% trajectory plot
lla = ecef2lla(X(1:3,:)');
figure()
geoplot(lla(:,1), lla(:,2), 'lineWidth', 2);
hold on
geoplot(rtklla(1,:), rtklla(2,:), 'lineWidth', 2);
geoplot(gpslla(:,1), gpslla(:,2), 'lineWidth', 2);
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



