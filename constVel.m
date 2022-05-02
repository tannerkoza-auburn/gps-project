%{
    Constant velocity tightly-coupled EKF
%}

clear; clc; close all;


%% INIT
% init at center of test track for testing:
lla = [32.595544, -85.295268, 700]; % deg, deg, m
ECEF = lla2ecef(lla);

% init state vector
pos(:,1) = ECEF';
vel(:,1) = [0; 0; 0];
clkBias(1) = 0;
clkDrift(1) = 0;
X(:,1) = [pos; vel; clkBias(1); clkDrift(1)];


%% TUNING
P(:,:,1) = eye(8,8);
Q = eye(8,8);


