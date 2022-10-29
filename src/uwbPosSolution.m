%% UWB Augmented GPS Solutions - GPS Project
clear
clc
close all

%% figure8 Run (All SVs 1 UWB)

% Import Data
load('roverData.mat')
load('simUWBs.mat')
load('figure8_rtk.mat')

% Instantiate Receiver Class
r = gnssReceiver();
% r.initPos = [4.412163140000000e+05; -5.360781415000000e+06; 3.416370097000000e+06];

% Base Position Initialization
base102Pos = figure8UWB.base102.svPos;
base103Pos = figure8UWB.base103.svPos;
basePseudoPos = figure8UWB.basePseudo.svPos;

cn0Thresh = 0;

C = physconst('LightSpeed');

% Number of Measurements
numMes = length(figure8.gpsRawObs);

for i = 1:numMes

    prnIdx = find(figure8.gpsRawObs(i).L1.cn0 > cn0Thresh);

    psr = [figure8.gpsRawObs(i).L1.psr(prnIdx)];% figure8UWB.base102.rngTrue(i) figure8UWB.base103.rngTrue(i) figure8UWB.basePseudo.rngTrue(i)];

    svPosX = [figure8.gpsSV(i).posX(prnIdx)]; %base102Pos(1) base103Pos%(1) basePseudoPos(1)];
    svPosY = [figure8.gpsSV(i).posY(prnIdx)];% base102Pos(2) base103Pos(2) basePseudoPos(2)];
    svPosZ = [figure8.gpsSV(i).posZ(prnIdx)]; %base102Pos(3) base103Pos(3) basePseudoPos(3)];
    svPos = [svPosX' svPosY' svPosZ'];

    svClockCorr = [figure8.gpsSV(i).svClockCorr(prnIdx)];

    figure8Pos = r.p3D(psr,svPos,svClockCorr);

    llaL(i,:) = ecef2lla(figure8Pos.pos');
    prnLL(i) = length(psr);

end

geoplot(llaL(:,1),llaL(:,2),'*')
geobasemap satellite
hold on
geoplot(data.lla(1,:),data.lla(2,:),'*')