clear
clc

load('roverData.mat')

r = gnssReceiver();

psr = random.gpsRawObs(1).L1.psr;
dopp = random.gpsRawObs(1).L1.dopp;
svPosX = random.gpsSV(1).posX;
svPosY = random.gpsSV(1).posY;
svPosZ = random.gpsSV(1).posZ;
svPos = [svPosX' svPosY' svPosZ'];
svVelX = random.gpsSV(1).velX;
svVelY = random.gpsSV(1).velY;
svVelZ = random.gpsSV(1).velZ;
svVel = [svVelX' svVelY' svVelZ'];

svClockCorr = random.gpsSV(1).svClockCorr;

posSol = r.pv3D(psr,dopp,svPos,svVel,svClockCorr,1);

posLLA = ecef2lla(posSol.pos');

figure
geoplot(random.gpsLLA(1).lat,random.gpsLLA(1).lon,'*')
hold on
geoplot(posLLA(1),posLLA(2),'*')
geobasemap satellite

