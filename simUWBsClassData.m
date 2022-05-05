%{  
    Simulate UWB ranges on class rtk path 
%}

clc; clear; close all;

%% old lab data
load('RCVR1.mat'); % novatel on car
load('novatel_2022-01-26-13-24-45')

stop = 2841;

% trim the data to the start of bag
novIdx = find((RCVR1{1,1}.L1.gpsTime - data.gpsSeconds) == 0);
data.lla = data.lla(:, novIdx:stop);
data.ecef = data.ecef(:, novIdx:stop);
data.gpsSeconds = data.gpsSeconds(:, novIdx:stop);
timeVec = data.gpsSeconds - data.gpsSeconds(1);

rtklla = data.lla;

% antenna on roof true location
ecef_t(:,1) = [423203.359;
            -5361678.541; 
            3417280.681];
% ecef_t(:,2) =  ecef_t(:,1) + [10000; 0; 0]; good ones
% ecef_t(:,3) =  ecef_t(:,1) + [0; 10000; 0]; good ones
lla = ecef2lla(ecef_t(:,1)');

% keep the same altitude for each (planar)
lla2 = [32.599719 -85.491692]; % outside coliseum
lla3 = [32.601662 -85.483173];
ecef_t(:,2) = lla2ecef([lla2(1) lla2(2) lla(3)]);
ecef_t(:,3) = lla2ecef([lla3(1) lla3(2) lla(3)]);

% upsample RTK
Hz = 1; % desired update rate (of UWBs)
dt = 1/Hz;
upTime = 0:dt:timeVec(end) - dt;
rtkllaUP(1,:) = interp1(timeVec, rtklla(1,:), upTime);
rtkllaUP(2,:) = interp1(timeVec, rtklla(2,:), upTime);
rtkllaUP(3,:) = interp1(timeVec, rtklla(3,:), upTime);
ecefUP(1,:) = interp1(timeVec, data.ecef(1,:), upTime);
ecefUP(2,:) = interp1(timeVec, data.ecef(2,:), upTime);
ecefUP(3,:) = interp1(timeVec, data.ecef(3,:), upTime);

UWBsigma = 5; % 10 cm

for i = 1:length(ecefUP)
    uwbRange(i,1) = norm(ecefUP(:,i) - ecef_t(:,1));
    uwbRange(i,2) = norm(ecefUP(:,i) - ecef_t(:,2));
    uwbRange(i,3) = norm(ecefUP(:,i) - ecef_t(:,3));
    
    uwbRange(i,:) = uwbRange(i,:) + UWBsigma*randn(1,3);
    uwbTime(i) = upTime(i);
end 



uwbPos(1,:) = ecef_t(:,1)';
uwbPos(2,:) = ecef_t(:,2)';
uwbPos(3,:) = ecef_t(:,3)';

save('UWBclassData.mat', 'uwbRange', 'uwbPos', 'uwbTime', 'UWBsigma');