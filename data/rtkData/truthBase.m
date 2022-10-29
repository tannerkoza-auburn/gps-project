%% Base Truth Position
clear
clc
close all

%% Base 102

base102 = load('base_102.mat');

base102avg = [mean(base102.data.lla(1,:)) mean(base102.data.lla(2,:))];

figure
geoplot(base102avg(1),base102avg(2),'g*')
hold on

%% Base 103

base103 = load('base_103.mat');
base103avg = [mean(base103.data.lla(1,:)) mean(base103.data.lla(2,:))];

geoplot(base103avg(1),base103avg(2),'c*')

%% Base Pseudo

basePseudo = load('base_pseudo.mat');
basePseudoavg = [mean(basePseudo.data.lla(1,:)) mean(basePseudo.data.lla(2,:))];

geoplot(basePseudoavg(1),basePseudoavg(2),'m*')
geobasemap satellite
title('UWB Base RTK Truth')
legend('Base 102','Base 103','Base Pseudo', ...
    'Location','northwest')