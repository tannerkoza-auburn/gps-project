clear
clc
close all

%% Implementation

% Import Data
load('simUWBs.mat')
load('figure8_rtk.mat')

% Instatiafigure8te Receiver Class
r = gnssReceiver();
r.initPos = [4.412175000000000e+05; -5.360781415000000e+06; 3.416370097000000e+06];

% Base Station Initialization
base102 = figure8UWB.base102.svPos;
base103 = figure8UWB.base103.svPos;
baseP = figure8UWB.basePseudo.svPos;

% Number of UWB Ranges
numMes = length(figure8UWB.base102.rngNoisy);

for i = 1:numMes
    
    psr = [figure8UWB.base102.rngNoisy(i) ...
        figure8UWB.base103.rngNoisy(i) ...
        figure8UWB.basePseudo.rngNoisy(i)];

    basePos = [base102; base103; baseP];

    pos = r.p3DPC(psr,basePos');

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
