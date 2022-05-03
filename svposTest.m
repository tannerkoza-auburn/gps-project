clear
clc

load('roverData.mat')

r = gnssReceiver();

psr = static.gpsRawObs(1).L1.psr;
dopp = static.gpsRawObs(1).L1.psr;
svPosX = static.gpsSV(1).posX;
svPosY = static.gpsSV(1).posY;
svPosZ = static.gpsSV(1).posZ;
svPos = [svPosX' svPosY' svPosZ'];
svVel = zeros(length(svPosX),3);
svClockCorr = static.gpsSV(1).svClockCorr;

posSol = r.p3D(psr,svPos,svClockCorr);

figure
geoplot(static.gpsLLA(1).lat,static.gpsLLA(1).lon,'*')
hold on
