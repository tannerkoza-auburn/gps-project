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

C = physconst('LightSpeed');

numMes = length(figure8.gpsRawObs);

for i = 1:numMes
    % SV in View
    svInView = figure8.gpsRawObs(i).L1.prn;
    numPRN = length(svInView);

    % Extract Ephem for SV in View
    ephem = gps_ephem(svInView);

    % Transit & Transmit Time
    transitTime = figure8.gpsRawObs(i).L1.psr/C;
    transmitTime = figure8.gpsTime(i).gpsS - transitTime;

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
    figure8.gpsSV(i).gpsS = figure8.gpsTime(i).gpsS;
    figure8.gpsSV(i).prn = svInView;
    figure8.gpsSV(i).posX = svPos(1,:);
    figure8.gpsSV(i).posY = svPos(2,:);
    figure8.gpsSV(i).posZ = svPos(3,:);
    figure8.gpsSV(i).velX = svVel(1,:);
    figure8.gpsSV(i).velY = svVel(2,:);
    figure8.gpsSV(i).velZ = svVel(3,:);
    figure8.gpsSV(i).svClockCorr = svClockCorr;
end

save('data/roverData/mats/roverData.mat','figure8','-append')