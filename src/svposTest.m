clear
clc

load('roverData.mat')

r = gnssReceiver();
i = 1;

psr = random.gpsRawObs(i).L1.psr;
dopp = random.gpsRawObs(i).L1.dopp;
svPosX = random.gpsSV(i).posX;
svPosY = random.gpsSV(i).posY;
svPosZ = random.gpsSV(i).posZ;
svPos = [svPosX' svPosY' svPosZ'];
svVelX = random.gpsSV(i).velX;
svVelY = random.gpsSV(i).velY;
svVelZ = random.gpsSV(i).velZ;
svVel = [svVelX' svVelY' svVelZ'];

svClockCorr = random.gpsSV(i).svClockCorr;

posSol = r.pv3D(psr,dopp,svPos,svVel,svClockCorr,1);

posLLA = ecef2lla(posSol.pos');

figure
geoplot(random.gpsLLA(1).lat,random.gpsLLA(1).lon,'*')
hold on
geoplot(posLLA(1),posLLA(2),'*')
geobasemap satellite

