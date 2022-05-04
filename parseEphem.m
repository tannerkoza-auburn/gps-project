%% Parse Ephemeris
% clear
% clc

%% Unpack SV Positions: Random Dataset

% Import Data
load('ephemData.mat')
load('roverData.mat')

C = physconst('LightSpeed');

numMes = length(random.gpsRawObs);

for i = 1:numMes
    % SV in View
    svInView = random.gpsRawObs(i).L1.prn;
    numPRN = length(svInView);

    % Extract Ephem for SV in View
    ephem = gps_ephem(svInView);

    % Transit & Transmit Time
    transitTime = random.gpsRawObs(i).L1.psr/C;
    transmitTime = random.gpsTime(i).gpsS - transitTime;

    % Preallocation
    svPos = zeros(3,numPRN);
    svVel = zeros(3,numPRN);
    svClockCorr = zeros(1,numPRN);

    % Calculate SV Parameters
    for j = 1:numPRN
        [svPos(:,j), svVel(:,j), svClockCorr(1,j)] = ...
            calc_sv_pos(ephem(j), transmitTime(j), transitTime(j));
    end

    % Populate Output Struct
    random.gpsSV(i).gpsS = random.gpsTime(i).gpsS;
    random.gpsSV(i).prn = svInView;
    random.gpsSV(i).posX = svPos(1,:);
    random.gpsSV(i).posY = svPos(2,:);
    random.gpsSV(i).posZ = svPos(3,:);
    random.gpsSV(i).velX = svVel(1,:);
    random.gpsSV(i).velY = svVel(2,:);
    random.gpsSV(i).velZ = svVel(3,:);
    random.gpsSV(i).svClockCorr = svClockCorr;
end

save('data/roverData/mats/roverData.mat','random','-append')