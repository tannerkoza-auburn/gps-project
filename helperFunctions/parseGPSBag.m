function out = parseGPSBag(pathName,fileName,gpsTopics)
%% Import Bag

bag = rosbag(strcat(pathName, fileName));
numTopics = length(gpsTopics);

%% Parse GPS Bag

for i = 1:numTopics
    % Current Topic
    currTopic = gpsTopics(i);
    bagSelect = select(bag, 'Topic', currTopic);
    msg =  readMessages(bagSelect,'dataFormat','struct');
    msgType = bagSelect.MessageList.MessageType(1);
    numMes = bagSelect.NumMessages;

    switch msgType
        case 'ros_sensor_msgs/GpsTimeTagged'
            for j = 1:numMes
                % Message Time
                out.gpsTime(j).msgTimeS = msg{j}.Header.Stamp.Sec;
                out.gpsTime(j).msgTimeNS = msg{j}.Header.Stamp.Nsec;
                out.gpsTime(j).msgTime = out.gpsTime(j).msgTimeS ...
                    + (out.gpsTime(j).msgTimeNS.*1e-9);

                % GPS Time
                out.gpsTime(j).gpsWeek = msg{j}.GpsTime.GpsWeek;
                out.gpsTime(j).gpsS = msg{j}.GpsTime.GpsSeconds;
            end

        case 'ros_sensor_msgs/GpsEphemerisTagged'
            for j = 1:numMes
                % Message Time
                out.gpsEphem(j).msgTimeS = msg{j}.Header.Stamp.Sec;
                out.gpsEphem(j).msgTimeNS = msg{j}.Header.Stamp.Nsec;
                out.gpsEphem(j).msgTime = out.gpsEphem(j).msgTimeS ...
                    + (out.gpsEphem(j).msgTimeNS.*1e-9);

                % GPS Ephemeris
                out.gpsEphem(j).prn = msg{j}.Ephemeris.Prn;
                out.gpsEphem(j).subframe1 = msg{j}.Ephemeris.Subframe1;
                out.gpsEphem(j).subframe2 = msg{j}.Ephemeris.Subframe2;
                out.gpsEphem(j).subframe3 = msg{j}.Ephemeris.Subframe3;
            end

        case 'ros_sensor_msgs/LlhPositionTagged'
            for j = 1:numMes
                % Message Time
                out.gpsLLA(j).msgTimeS = msg{j}.Header.Stamp.Sec;
                out.gpsLLA(j).msgTimeNS = msg{j}.Header.Stamp.Nsec;
                out.gpsLLA(j).msgTime = out.gpsLLA(j).msgTimeS ...
                    + (out.gpsLLA(j).msgTimeNS.*1e-9);

                % GPS LLA
                out.gpsLLA(j).lat = msg{j}.LlhPosition.Latitude;
                out.gpsLLA(j).lon = msg{j}.LlhPosition.Longitude;
                out.gpsLLA(j).alt = msg{j}.LlhPosition.Altitude;
            end

        case 'ros_sensor_msgs/RawMeasurementsTagged'
            for j = 1:numMes
                % Message Time
                out.gpsRawObs(j).msgTimeS = msg{j}.Header.Stamp.Sec;
                out.gpsRawObs(j).msgTimeNS = msg{j}.Header.Stamp.Nsec;
                out.gpsRawObs(j).msgTime = out.gpsRawObs(j).msgTimeS ...
                    + (out.gpsRawObs(j).msgTimeNS.*1e-9);

                % GPS Raw Observables
                out.gpsRawObs(j).prn = extractfield(msg{j}.Measurements,'Prn');
                out.gpsRawObs(j).psr = extractfield(msg{j}.Measurements,'Pseudorange');
                out.gpsRawObs(j).psr_variance = extractfield(msg{j}.Measurements,'PseudorangeVariance');

                for k = 1:length(out.gpsRawObs(j).prn)
                    fields = fieldnames(msg{j}.Measurements(k).Frequency);
                    type = msg{j}.Measurements(k).Frequency.Type;

                    out.gpsRawObs(j).frequencyType(k) = string(fields{type + 2});
                end

                out.gpsRawObs(j).carr_phase = extractfield(msg{j}.Measurements,'CarrierPhase');
                out.gpsRawObs(j).carr_phase_variance = extractfield(msg{j}.Measurements,'CarrierPhaseVariance');
                out.gpsRawObs(j).dopp = extractfield(msg{j}.Measurements,'Doppler');
                out.gpsRawObs(j).dopp_variance = extractfield(msg{j}.Measurements,'DopplerVariance');
                out.gpsRawObs(j).cn0 = extractfield(msg{j}.Measurements,'CarrierToNoise');
                out.gpsRawObs(j).loss_of_lock = cell2mat(extractfield(msg{j}.Measurements,'LossOfLock'));
            end
    end
end