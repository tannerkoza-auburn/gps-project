%% Rover Truth Position
clear
clc
close all

%% Static Run

% static = load('static_rtk.mat');
% 
% figure
% geoplot(static.data.lla(1,:),static.data.lla(2,:),'*')
% geobasemap satellite
% title('Static Rover RTK Truth')

%% Dynamic Figure 8 Run

dynamicF8 = load('figure8_rtk.mat');

figure
geoplot(dynamicF8.data.lla(1,:),dynamicF8.data.lla(2,:),'*')
hold on
geoplot(dynamicF8.data.lla(1,1),dynamicF8.data.lla(2,1),'g*')
geoplot(dynamicF8.data.lla(1,end),dynamicF8.data.lla(2,end),'r*')
geobasemap satellite
title('Dynamic Rover RTK Truth: Figure 8')
legend('Dynamic Truth Position','Start Position','End Position', ...
    'Location','northwest')

%% Dynamic Random Run

dynamicRand = load('random_rtk.mat');

figure
geoplot(dynamicRand.data.lla(1,:),dynamicRand.data.lla(2,:),'*')
hold on
geoplot(dynamicRand.data.lla(1,1),dynamicRand.data.lla(2,1),'g*')
geoplot(dynamicRand.data.lla(1,end),dynamicRand.data.lla(2,end),'r*')
geobasemap satellite
title('Dynamic Rover RTK Truth: Random')
legend('Dynamic Truth Position','Start Position','End Position', ...
    'Location','northwest')

%% Dynamic Bad Run

dynamicBad = load('straight_line_bad_rtk.mat');

figure
geoplot(dynamicBad.data.lla(1,:),dynamicBad.data.lla(2,:),'*')
hold on
geoplot(dynamicBad.data.lla(1,1),dynamicBad.data.lla(2,1),'g*')
geoplot(dynamicBad.data.lla(1,end),dynamicBad.data.lla(2,end),'r*')
geobasemap satellite
title('Dynamic Rover RTK Truth: Bad')
legend('Dynamic Truth Position','Start Position','End Position', ...
    'Location','northwest')