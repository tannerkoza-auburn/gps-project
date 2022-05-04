%% UWB Augmented GPS Solutions - GPS Project
clear
clc
close all

%% Random Run (All SVs 1 UWB)

% Import Data
load('roverData.mat')
load('simUWBs.mat')

% Instantiate Receiver Class
r = gnssReceiver();
% r.initPos = [4.412163140000000e+05; -5.360781415000000e+06; 3.416370097000000e+06];

% Base Position Initialization
base102Pos = randomUWB.base102.svPos;
base103Pos = randomUWB.base103.svPos;
basePseudoPos = randomUWB.basePseudo.svPos;

% Number of Measurements 
numMes = length(random.gpsRawObs);

for i = 1:numMes

    psr = [random.gpsRawObs(i).L1.psr ...
        randomUWB.base102.rngNoisy(i)];
    
    svPosX = [random.gpsSV(i).posX base102Pos(1)];
    svPosY = [random.gpsSV(i).posY base102Pos(2)];
    svPosZ = [random.gpsSV(i).posZ base102Pos(3)];
    svPos = [svPosX' svPosY' svPosZ'];

    svClockCorr = [random.gpsSV(i).svClockCorr 0];

    randomPos = r.p3D(psr,svPos,svClockCorr)

end