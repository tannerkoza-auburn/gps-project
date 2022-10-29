%% Simulate UWB Ranges - GPS Project
clear
clc
close all

%% Simulation Initialization

% Import Static & Base Data
load('roverData.mat')
base102 = load('base_102.mat');
base103 = load('base_103.mat');
basePseudo = load('base_pseudo.mat');

% Average Static Position
% NOTE: This is necessary because the RTK solution was unavailable.
staticLLA = [vertcat(static.gpsLLA.lat) vertcat(static.gpsLLA.lon) ...
    vertcat(static.gpsLLA.alt)];
staticECEF = lla2ecef(staticLLA);
staticMean = mean(staticECEF);

% Average Base 102 RTK Position
base102ECEF = base102.data.ecef';
base102Mean = mean(base102ECEF);
figure8UWB.base102.svPos = base102Mean;
randomUWB.base102.svPos = base102Mean;

% Average Base 103 RTK Position
base103ECEF = base103.data.ecef';
base103Mean = mean(base103ECEF);
figure8UWB.base103.svPos = base103Mean;
randomUWB.base103.svPos = base103Mean;

% Average Base Pseudo RTK Position
basePseudoECEF = basePseudo.data.ecef';
basePseudoMean = mean(basePseudoECEF);
figure8UWB.basePseudo.svPos = basePseudoMean;
randomUWB.basePseudo.svPos = basePseudoMean;

% Error Statistics
rngSigma = 0.1; % meters
sigmaSF = 1; % sigma scale factor

%% Figure 8 Range

% Import Figure 8 Data
figure8RTK = load('figure8_rtk.mat');

% Calculate True Ranges
figure8ECEF = figure8RTK.data.ecef';

figure8UWB.base102.rpv = base102Mean - figure8ECEF; % Base to Rover RPV
figure8UWB.base103.rpv = base103Mean - figure8ECEF; 
figure8UWB.basePseudo.rpv = basePseudoMean - figure8ECEF;

figure8UWB.base102.rngTrue = vecnorm(figure8UWB.base102.rpv,2,2); % True Ranges
figure8UWB.base103.rngTrue = vecnorm(figure8UWB.base103.rpv,2,2);
figure8UWB.basePseudo.rngTrue = vecnorm(figure8UWB.basePseudo.rpv,2,2);

rngIndices = find(ismember(vertcat(figure8RTK.data.gpsSeconds),vertcat(figure8.gpsTime.gpsS)))';
numRngs = length(rngIndices);
figure8UWB.base102.rngTrue = figure8UWB.base102.rngTrue(rngIndices);
figure8UWB.base103.rngTrue = figure8UWB.base103.rngTrue(rngIndices);
figure8UWB.basePseudo.rngTrue = figure8UWB.basePseudo.rngTrue(rngIndices);

figure8UWB.base102.rngNoisy = figure8UWB.base102.rngTrue ...
    + sigmaSF*rngSigma*randn(numRngs,1);
figure8UWB.base103.rngNoisy = figure8UWB.base103.rngTrue ...
    + sigmaSF*rngSigma*randn(numRngs,1);
figure8UWB.basePseudo.rngNoisy = figure8UWB.basePseudo.rngTrue ...
    + sigmaSF*rngSigma*randn(numRngs,1);

%% Random Range

% Import Radnom Data
randomRTK = load('random_rtk.mat');

% Calculate True Ranges
randomECEF = randomRTK.data.ecef';

randomUWB.base102.rpv = base102Mean - randomECEF; % Base to Rover RPV
randomUWB.base103.rpv = base103Mean - randomECEF; 
randomUWB.basePseudo.rpv = basePseudoMean - randomECEF;

randomUWB.base102.rngTrue = vecnorm(randomUWB.base102.rpv,2,2); % True Ranges
randomUWB.base103.rngTrue = vecnorm(randomUWB.base103.rpv,2,2);
randomUWB.basePseudo.rngTrue = vecnorm(randomUWB.basePseudo.rpv,2,2);

rngIndices = find(ismember(vertcat(randomRTK.data.gpsSeconds),vertcat(random.gpsTime.gpsS)))';
numRngs = length(rngIndices);
randomUWB.base102.rngTrue = randomUWB.base102.rngTrue(rngIndices);
randomUWB.base103.rngTrue = randomUWB.base103.rngTrue(rngIndices);
randomUWB.basePseudo.rngTrue = randomUWB.basePseudo.rngTrue(rngIndices);

randomUWB.base102.rngNoisy = randomUWB.base102.rngTrue ...
    + sigmaSF*rngSigma*randn(numRngs,1);
randomUWB.base103.rngNoisy = randomUWB.base103.rngTrue ...
    + sigmaSF*rngSigma*randn(numRngs,1);
randomUWB.basePseudo.rngNoisy = randomUWB.basePseudo.rngTrue ...
    + sigmaSF*rngSigma*randn(numRngs,1);

%% Save Simulated UWBs

save('data/simUWBData/simUWBs.mat','figure8UWB','randomUWB')
