%{
    Testing the results of ECEF Mechanization with kvh data from car
%}
clear all; close all; clc;
load('classIMU.mat');
load('RCVR1.mat');




%% Init
rcvr1 = gnssReceiver();
psr1 = RCVR1{1}.L1.psr;
dopp1 = RCVR1{1}.L1.dopp;
svPos1 = RCVR1{1}.L1.svPos;
svVel1 = RCVR1{1}.L1.svVel;
clkCorr1 = RCVR1{1}.L1.clkCorr;
est1 = rcvr1.pv3D(psr1,dopp1,svPos1,svVel1,clkCorr1, 1);

for i = 1:length(RCVR1)
    psr1 = RCVR1{i}.L1.psr;
    dopp1 = RCVR1{i}.L1.dopp;
    svPos1 = RCVR1{i}.L1.svPos;
    svVel1 = RCVR1{i}.L1.svVel;
    clkCorr1 = RCVR1{i}.L1.clkCorr;
    est(i) = rcvr1.pv3D(psr1,dopp1,svPos1,svVel1,clkCorr1, 1);
    
    gpsVel(:,i) = est(i).vel;
end 

% init state vector
pos(:,1) = est1.pos';
vel(:,1) = est1.vel;
clkBias(1) = est1.clock_bias;
clkDrift(1) = est1.clock_drfit;
X(:,1) = [pos; vel; clkBias(1); clkDrift(1)];

lla = ecef2lla(est1.pos'); % init lla
% init rotation matrix C_b2e
Eul(:,1) = [0; 0; -90]; % import init heading in deg
[C_n2b, C_b2n(:,:,1)] = NED_to_Body(Eul);
[C_n2e] = NED_to_ECEF(lla(1), lla(2));
C_b2e(:,:,1) = C_n2e*C_b2n;

%---- init NED
NED(:,1) = [0; 0; 0];
vNED(:,1) = [0; 0; 0];
lat(1) = lla(1);
lon(1) = lla(2);
h(1) = lla(3);



%% TUNING
P(:,:,1) = est1.P;
Q = 100*eye(8,8);


%% Main Loop

% kvh.IMU(:,1) = kvh.IMU(:,1) - kvh.IMU(1,1); % reset time

for i = 1:length(kvh.IMU)
    
    % store old PVA
    oldPVA.pos = pos(:,i);
    oldPVA.vel = vel(:,i);
    oldPVA.C_b2e = C_b2e(:,:,i);
    
    % extract meas and find dt:
    f_ib_b = kvh.IMU(i,2:4)'; %  - aBias;
    omega_ib_b = kvh.IMU(i,5:7)';
    dt = kvh.IMU(i,1) - kvh.IMU(max([i-1,1]),1);
    
    % mechanize ECEF
    [pos(:,i+1), vel(:,i+1), C_b2e(:,:,i+1)] = Mechanize_ECEF(f_ib_b, omega_ib_b, dt, oldPVA);
    
    % mechanize NED
    [lat(i+1), lon(i+1), h(i+1), NED(:,i+1), vNED(:,i+1), C_b2n(:,:,i+1)] = Mechanize_NED(f_ib_b, omega_ib_b, lat(i), lon(i), h(i),...
        NED(:,i), vNED(:,i), C_b2n(:,:,i), dt);
    
    eul = rad2deg(rotm2eul(C_b2n(:,:,i))); % extract Euler angles of misalign
    mis_est(:,i) = [eul(3); eul(2); eul(1)]; % store current misalign est
    
end 

lla_out = ecef2lla(pos');

figure()
geoplot(lla_out(:,1), lla_out(:,2), '*')
hold on 
geoplot(lat, lon, '*')
geobasemap satellite

figure()
geoplot(lat, lon, '*')
hold on
geobasemap satellite

figure()
plot(mis_est(1,:))
hold on
plot(mis_est(2,:))
plot(mis_est(3,:))
% 
% figure()
% plot(Mkz.gpsHeading(:,1), Mkz.gpsHeading(:,2))


figure()
plot(Mkz.gpsGyro(:,1), deg2rad(Mkz.gpsGyro(:,4)), '*');
hold on 
plot(kvh.IMU(:,1), kvh.IMU(:,7));

figure()
subplot(3,1,1)
plot(NED(1,:))
title('North')
subplot(3,1,2)
plot(NED(2,:))
title('East')
subplot(3,1,3)
plot(NED(3,:))
title('Down')

figure()
subplot(2,1,1)
plot(kvh.accel(:,1))
hold on 
title('Body-frame accel')
plot(kvh.accel(:,2))
plot(kvh.accel(:,3))
ylabel('m/s^2')
legend('x','y','z')
subplot(2,1,2)
plot(kvh.gyro(:,1))
hold on 
title('Body-frame gyro')
plot(kvh.gyro(:,2))
ylabel('rad/s')
plot(kvh.gyro(:,3))
legend('x','y','z')


