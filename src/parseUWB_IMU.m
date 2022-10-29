%{
    Script for parsing the UWB base-station ranges and the KVH on the MKZ
    for final project of GPS
%} 
close all; clc; clear;

fileName = 'figure8';
stringMkz = sprintf('%s.bag', fileName);
bagMkz = rosbag(stringMkz);

etalin_hdg_rotate = 1; % flag for if etalin needs to be rotated 180 deg


%{
    KVH is oriented as North West Up
    
    x is along body
    y is out left windows
    z is up 

    NED is
    x along body
    y out right windows
    z is down

    Result:
    - invert KVH y and z accel measurements
    - invert KVH yaw and pitch gyro
%}



%% --- Parse Etalin for pos and heading
etalinMkz.hdg = select(bagMkz, 'Topic', '/etalin/attitude');
etalinMkz.hdg = readMessages(etalinMkz.hdg, 'DataFormat', 'struct');
for i = 1:length(etalinMkz.hdg)      % extract heading and header
    
    % extract heading
    Mkz.gpsHeading(i,2) = wrapTo180(double(etalinMkz.hdg{i,1}.BodyAttitude.Yaw));
    
    if etalin_hdg_rotate
        Mkz.gpsHeading(i,2) = wrapTo180(double(etalinMkz.hdg{i,1}.BodyAttitude.Yaw) + 180);
    end 
    
    % associated ROS time
    Mkz.gpsHeading(i,1) = double(etalinMkz.hdg{i,1}.Header.Stamp.Sec + ...
                         etalinMkz.hdg{i,1}.Header.Stamp.Nsec * 10^-9);
    
end 

etalinMkz.pos = select(bagMkz, 'Topic', '/etalin/odom');
etalinMkz.pos = readMessages(etalinMkz.pos, 'DataFormat', 'struct');
for i = 1:length(etalinMkz.pos)
    
    Mkz.gpsECEF(i,2) = etalinMkz.pos{i,1}.Pose.Pose.Position.X;
    Mkz.gpsECEF(i,3) = etalinMkz.pos{i,1}.Pose.Pose.Position.Y;
    Mkz.gpsECEF(i,4) = etalinMkz.pos{i,1}.Pose.Pose.Position.Z;
    Mkz.gpsECEF(i,1) = double(etalinMkz.pos{i,1}.Header.Stamp.Sec + ...
                         etalinMkz.pos{i,1}.Header.Stamp.Nsec * 10^-9);
end 


etalinMkz.gyro = select(bagMkz, 'Topic', 'etalin/angular_velocity');
etalinMkz.gyro = readMessages(etalinMkz.gyro, 'DataFormat', 'struct');
for i = 1:length(etalinMkz.gyro)
    
    Mkz.gpsGyro(i,2) = etalinMkz.gyro{i,1}.AngularVelocity.Roll;
    Mkz.gpsGyro(i,3) = etalinMkz.gyro{i,1}.AngularVelocity.Pitch;
    Mkz.gpsGyro(i,4) = etalinMkz.gyro{i,1}.AngularVelocity.Yaw;
    Mkz.gpsGyro(i,1) = double(etalinMkz.gyro{i,1}.Header.Stamp.Sec + ...
                         etalinMkz.gyro{i,1}.Header.Stamp.Nsec * 10^-9);
end 





%% --- Ranging Measurements
% select ranges from node 100 and sort by destinations 102 and 103
range.select = select(bagMkz, 'Topic', '/rangenet_node_100/range');
range.struct = readMessages(range.select, 'DataFormat', 'struct');
range.range = cellfun(@(m) double(m.PrecisionRange)/1000, range.struct);
range.err = cellfun(@(m) double(m.PrecisionRangeErrEst)/1000, range.struct);
range.time = cellfun(@(m) double(m.Header.Stamp.Nsec)*10^-9 + ...
    double(m.Header.Stamp.Sec), range.struct);
range.DestId = cellfun(@(m) double(m.DestId), range.struct);
range.array = [range.time 100*ones(length(range.DestId),1) range.DestId range.range range.err];
rng_100_102 = range.array( (find(range.array(:,3) == 102)), 4:5);
rng_100_103 = range.array( (find(range.array(:,3) == 103)), 4:5);


% throwing out the invalid ranges
idx = find(range.array(:,4) == 0); % find dropped ranged
range.array(idx,:) = []; % erase empty range rows
rng_100_102(rng_100_102(:,1) == 0) = NaN;
rng_100_103(rng_100_103(:,1) == 0) = NaN;

figure()
plot(rng_100_102(:,1));
hold on 
plot(rng_100_103(:,1));
legend('Base 102', 'Base 103')

% add extra columns of fluff to match in size to the imu meas
range.array = [range.array zeros(length(range.array),2)];




%% --- IMU Measurements
kvh.select = select(bagMkz, 'Topic', '/kvh/imu');
kvh.struct = readMessages(kvh.select, 'DataFormat', 'struct');
kvh.time = cellfun(@(m) double(m.Header.Stamp.Nsec)*10^-9 + ...
    double(m.Header.Stamp.Sec), kvh.struct);
kvh.accel(:,1) = cellfun(@(m) double(m.LinearAcceleration.X), kvh.struct);
kvh.accel(:,2) = -cellfun(@(m) double(m.LinearAcceleration.Y), kvh.struct);
kvh.accel(:,3) = -cellfun(@(m) double(m.LinearAcceleration.Z), kvh.struct);
kvh.gyro(:,1) = cellfun(@(m) double(m.AngularVelocity.X), kvh.struct);
kvh.gyro(:,2) = -cellfun(@(m) double(m.AngularVelocity.Y), kvh.struct);
kvh.gyro(:,3) = -cellfun(@(m) double(m.AngularVelocity.Z), kvh.struct);

kvh.IMU = [kvh.time kvh.accel kvh.gyro];


save('data/roverData/mats/figure8_IMU.mat', 'kvh', 'Mkz');


%% --- sort IMU/UWB measurements in time:

% sort with ROS time
Meas_Matrix = [kvh.IMU; range.array];
Meas_Matrix = sortrows(Meas_Matrix);
Meas_Matrix(:,1) = Meas_Matrix(:,1) - Meas_Matrix(1,1); % reset time to zero


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


