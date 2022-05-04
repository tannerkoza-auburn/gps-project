%{
Take in UWB mats, satellite mats, and IMU mats and combine into
measurements synced with rosTime. Meas Matrix contains flags for which
measurements to use.
%}
clear; close all; clc;
load('roverData')
load('figure8_IMU')
load('simUWBs');

IMUmat = kvh.IMU;
svStruct = figure8;
camIMUmat = camIMU.IMU;

for i = 1:length(svStruct.gpsSV)
    
    svMat(i).time = svStruct.gpsRawObs(i).msgTime;   
    svMat(i).psr = svStruct.gpsRawObs(i).L1.psr;
    svMat(i).psrVar = svStruct.gpsRawObs(i).L1.psr_variance;
    svMat(i).dopp = svStruct.gpsRawObs(i).L1.dopp;
    svMat(i).doppVar = .5*ones(length(svMat(i).dopp),1); % placeholder bc 0 doppvar
    svMat(i).svPos(:,1) = svStruct.gpsSV(i).posX';
    svMat(i).svPos(:,2) = svStruct.gpsSV(i).posY';
    svMat(i).svPos(:,3) = svStruct.gpsSV(i).posZ';
    svMat(i).svVel(:,1) = svStruct.gpsSV(i).velX';
    svMat(i).svVel(:,2) = svStruct.gpsSV(i).velY';
    svMat(i).svVel(:,3) = svStruct.gpsSV(i).velZ';
    svMat(i).clkCorr = svStruct.gpsSV(i).svClockCorr;
    
    svSyncMat(i,:) = [cast(svMat(i).time,'double') 1];
end 
IMUsyncMat = [IMUmat(:,1) zeros(length(IMUmat(:,1)),1)];
CAMsyncMat = [camIMUmat(:,1) zeros(length(camIMUmat(:,1)),1)];

timeSortKVH = sortrows([IMUsyncMat; svSyncMat]);
IMUmat(:,1) = IMUmat(:,1) - timeSortKVH(1,1); % reset IMU to 0
timeSortKVH(:,1) = timeSortKVH(:,1) - timeSortKVH(1,1);

% find indices of timeSort which correspond to the sv measurements
svMeasIdx(:,1) = find(timeSortKVH(:,2) == 1);

% reset time and store indices for satellite updates
for i = 1:length(svStruct.gpsSV)
    svMat(i).svIdx = svMeasIdx(i);
    svMat(i).time = timeSortKVH(svMeasIdx(i),1);
end 

% repeat for cam IMU
timeSortCam = sortrows([CAMsyncMat; svSyncMat]);
camIMUmat(:,1) = camIMUmat(:,1) - timeSortCam(1,1);
timeSortCam(:,1) = timeSortCam(:,1) - timeSortCam(1,1);

save('data/combinedMats/figure8_CombinedMats', 'svMat', 'IMUmat', 'camIMUmat', 'timeSortKVH', 'timeSortCam');

