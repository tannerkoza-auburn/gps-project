%% Rover Observables
clear
clc

%% Directory Initialization

inDir = 'data/roverData/bags/';
outDir = 'data/roverData/mats/';
gpsTopics = {'novatel/llhPositionTagged', ...
   'novatel/gpsTimeTagged', ...
   'novatel/gpsEphemerisTagged', ...
   'novatel/rawMeasurementsTagged', ...
   'novatel/svStateTagged'};

%% Static Data

staticBag = 'static_2022-05-02-14-04-44.bag';
static = parseGPSBag(inDir,staticBag,gpsTopics);

%% Dynamic Random Data

randomBag = 'random_2022-05-02-14-06-44.bag';
random = parseGPSBag(inDir,randomBag,gpsTopics);

%% Dynamic Figure 8 Data

figure8Bag = 'figure8_2022-05-02-13-59-18.bag';
figure8 = parseGPSBag(inDir,figure8Bag,gpsTopics);

%% Dynamic Straight Line Bad Data

straightBadBag = 'straight_line_bad_2022-05-02-14-02-19.bag';
straightBad = parseGPSBag(inDir,straightBadBag,gpsTopics);

%% Save Data

save(strcat(outDir,'roverData'),'static','random','figure8','straightBad')
