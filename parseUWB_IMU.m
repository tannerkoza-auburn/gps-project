%{
    Script for parsing the UWB base-station ranges and the KVH on the MKZ
    for final project of GPS
%} 
close all; clc; clear;

fileName = 'random';
stringMkz = sprintf('%s.bag', fileName);
bagMkz = rosbag(stringMkz);



%% --- Ranging Measurements
% select ranges from node 100 and sort by destinations 102 and 103
range_100.select = select(bagMkz, 'Topic', '/rangenet_node_100/range');
range_100.struct = readMessages(range_100.select, 'DataFormat', 'struct');
range_100.range = cellfun(@(m) double(m.PrecisionRange)/1000, range_100.struct);
range_100.err = cellfun(@(m) double(m.PrecisionRangeErrEst)/1000, range_100.struct);
range_100.time = cellfun(@(m) double(m.Header.Stamp.Nsec)*10^-9 + ...
    double(m.Header.Stamp.Sec), range_100.struct);
range_100.DestId = cellfun(@(m) double(m.DestId), range_100.struct);
range_100.array = [range_100.time 100*ones(length(range_100.DestId),1) range_100.DestId range_100.range range_100.err];
rng_100_102 = range_100.array( (find(range_100.array(:,3) == 102)), 4:5);
rng_100_103 = range_100.array( (find(range_100.array(:,3) == 103)), 4:5);


% throwing out the invalid ranges
rng_100_102(rng_100_102(:,1) == 0) = NaN;
rng_100_103(rng_100_103(:,1) == 0) = NaN;

figure()
plot(rng_100_102(:,1));
hold on 
plot(rng_100_103(:,1));
legend('100-102', '100-103')

range_100.time = range_100.time - range_100.time(1);

% collect UWB ranging into single array:
% UWB_array = sortrows([range_100.array; range_101.array]);