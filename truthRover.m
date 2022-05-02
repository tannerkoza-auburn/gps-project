%% Rover Truth Position
clear
clc
close all

%% Static Run

static = load('static_rtk.mat');

figure
geoplot(static.data.lla(1,:),static.data.lla(2,:),'*')
geobasemap satellite
title('Static Rover RTK Truth')

%% Dynamic Run

dynamic = load('dynamic_rtk.mat');

figure
geoplot(dynamic.data.lla(1,:),dynamic.data.lla(2,:),'*')
hold on
geoplot(dynamic.data.lla(1,1),dynamic.data.lla(2,1),'g*')
geoplot(dynamic.data.lla(1,end),dynamic.data.lla(2,end),'r*')
geobasemap satellite
title('Dynamic Rover RTK Truth')
legend('Dynamic Truth Position','Start Position','End Position', ...
    'Location','northwest')

%% Dynamic 1 Side Run

dynamic1Side = load('dynamic_1side_rtk.mat');

figure
geoplot(dynamic1Side.data.lla(1,:),dynamic1Side.data.lla(2,:),'*')
hold on
geoplot(dynamic1Side.data.lla(1,1),dynamic1Side.data.lla(2,1),'g*')
geoplot(dynamic1Side.data.lla(1,end),dynamic1Side.data.lla(2,end),'r*')
geobasemap satellite
title('Dynamic Rover RTK Truth: 1 Side')
legend('Dynamic Truth Position','Start Position','End Position', ...
    'Location','northwest')

%% Dynamic Bad Run

dynamicBad = load('dynamic_bad_rtk.mat');

figure
geoplot(dynamicBad.data.lla(1,:),dynamicBad.data.lla(2,:),'*')
hold on
geoplot(dynamicBad.data.lla(1,1),dynamicBad.data.lla(2,1),'g*')
geoplot(dynamicBad.data.lla(1,end),dynamicBad.data.lla(2,end),'r*')
geobasemap satellite
title('Dynamic Rover RTK Truth: Bad')
legend('Dynamic Truth Position','Start Position','End Position', ...
    'Location','northwest')