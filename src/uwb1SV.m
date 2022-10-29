clear
clc
close all

%% Implementation

% Import Data
load('simUWBs.mat')
load('random_rtk.mat')
load('roverData.mat')

% Instatiate Receiver Class
r = gnssReceiver();
r.initPos = [4.412175000000000e+05; -5.360781415000000e+06; 3.416370097000000e+06];

% Base Station Initialization
base102 = randomUWB.base102.svPos;
base103 = randomUWB.base103.svPos;
baseP = randomUWB.basePseudo.svPos;

% Number of UWB Ranges
numMes = length(randomUWB.base102.rngNoisy);

% PRN Index
prnIDX = 1:6;

for i = 1:numMes
    
    psr = [random.gpsRawObs(i).L1.psr(prnIDX) ...
        randomUWB.base102.rngNoisy(i) ...
        randomUWB.base103.rngNoisy(i) ...
        randomUWB.basePseudo.rngNoisy(i)];
    
    svPos = [random.gpsSV(i).posX(prnIDX); ...
        random.gpsSV(i).posY(prnIDX); ...
        random.gpsSV(i).posZ(prnIDX)];
    basePos = [svPos'; base102; base103; baseP];

    svClockCorr = random.gpsSV(i).svClockCorr(prnIDX);
    clockCorr = [svClockCorr 0 0 0];

    pos = r.p3D(psr,basePos',clockCorr)

    posL(i,:) = pos.pos;

end

lla = ecef2lla(posL);

figure
geoplot(data.lla(1,:),data.lla(2,:),'c*')
hold on
geoplot(lla(:,1),lla(:,2),'b*')
geoplot(data.lla(1,1),data.lla(2,1),'g*')
geoplot(data.lla(1,end),data.lla(2,end),'r*')
geobasemap satellite
legend('RTK Truth','UWB Solution','Start','End')
