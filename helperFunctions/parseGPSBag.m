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
                temp.gpsEphem(j).msgTimeS = msg{j}.Header.Stamp.Sec;
                temp.gpsEphem(j).msgTimeNS = msg{j}.Header.Stamp.Nsec;
                temp.gpsEphem(j).msgTime = temp.gpsEphem(j).msgTimeS ...
                    + (temp.gpsEphem(j).msgTimeNS.*1e-9);

                % GPS Ephemeris
                temp.gpsEphem(j).prn = msg{j}.Ephemeris.Prn;
                temp.gpsEphem(j).subframe1 = msg{j}.Ephemeris.Subframe1;
                temp.gpsEphem(j).subframe2 = msg{j}.Ephemeris.Subframe2;
                temp.gpsEphem(j).subframe3 = msg{j}.Ephemeris.Subframe3;

                % Decode Subframes
                out.gpsEphem(j) = decodeEphem(temp.gpsEphem(j));

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

                % Number of Pseudoranges
                numPsr = length(msg{j}.Measurements);

                % Frequency Counters
                L1ctr = 1;
                L2ctr = 1;

                for k = 1:numPsr

                    fCarr = msg{j}.Measurements(k).Frequency.Type;

                    if fCarr == 1

                        % GPS Raw Observables
                        out.gpsRawObs(j).L1.prn(L1ctr) = msg{j}.Measurements(k).Prn;
                        out.gpsRawObs(j).L1.psr(L1ctr) = msg{j}.Measurements(k).Pseudorange;
                        out.gpsRawObs(j).L1.psr_variance(L1ctr) = msg{j}.Measurements(k).PseudorangeVariance;
                        out.gpsRawObs(j).L1.carr_phase(L1ctr) = msg{j}.Measurements(k).CarrierPhase;
                        out.gpsRawObs(j).L1.carr_phase_variance(L1ctr) = msg{j}.Measurements(k).CarrierPhaseVariance;
                        out.gpsRawObs(j).L1.dopp(L1ctr) = msg{j}.Measurements(k).Doppler;
                        out.gpsRawObs(j).L1.dopp_variance(L1ctr) = msg{j}.Measurements(k).DopplerVariance;
                        out.gpsRawObs(j).L1.cn0(L1ctr) = msg{j}.Measurements(k).CarrierToNoise;
                        out.gpsRawObs(j).L1.loss_of_lock(L1ctr) = msg{j}.Measurements(k).LossOfLock;

                        L1ctr = L1ctr + 1;

                    elseif fCarr == 2

                        % GPS Raw Observables
                        out.gpsRawObs(j).L2.prn(L2ctr) = msg{j}.Measurements(k).Prn;
                        out.gpsRawObs(j).L2.psr(L2ctr) = msg{j}.Measurements(k).Pseudorange;
                        out.gpsRawObs(j).L2.psr_variance(L2ctr) = msg{j}.Measurements(k).PseudorangeVariance;
                        out.gpsRawObs(j).L2.carr_phase(L2ctr) = msg{j}.Measurements(k).CarrierPhase;
                        out.gpsRawObs(j).L2.carr_phase_variance(L2ctr) = msg{j}.Measurements(k).CarrierPhaseVariance;
                        out.gpsRawObs(j).L2.dopp(L2ctr) = msg{j}.Measurements(k).Doppler;
                        out.gpsRawObs(j).L2.dopp_variance(L2ctr) = msg{j}.Measurements(k).DopplerVariance;
                        out.gpsRawObs(j).L2.cn0(L2ctr) = msg{j}.Measurements(k).CarrierToNoise;
                        out.gpsRawObs(j).L2.loss_of_lock(L2ctr) = msg{j}.Measurements(k).LossOfLock;

                        L2ctr = L2ctr + 1;

                    else
                        error("Novatel doesn't have L1 of L2!")
                    end
                end

            end

        case 'ros_sensor_msgs/SvStateTagged'

            prnCtr = 1;  
            epochCtr = 1; 
            previousTimeStamp = [];
            for ii = 1:numMes

                currentTimeStamp = msg{ii}.Tags.GpsTime.GpsSeconds;
                if ~isempty(previousTimeStamp)
                    % Check for non-zero dt
                    dt = currentTimeStamp - previousTimeStamp;

                    if dt ~= 0
                        % A new observation period has occured!!!
                        epochCtr = epochCtr + 1;
                        prnCtr = 1;

                    end

                end


                out.gpsSV(epochCtr).gpsSec(prnCtr) = msg{ii}.Tags.GpsTime.GpsSeconds;
                out.gpsSV(epochCtr).prn(prnCtr) = msg{ii}.SvState.Prn;
                out.gpsSV(epochCtr).posX(prnCtr) = msg{ii}.SvState.Position.X;
                out.gpsSV(epochCtr).posY(prnCtr) = msg{ii}.SvState.Position.Y;
                out.gpsSV(epochCtr).posZ(prnCtr) = msg{ii}.SvState.Position.Z;
                out.gpsSV(epochCtr).svClockCorr(prnCtr) = msg{ii}.SvState.SvClockCorrection;

                % Finshing extracting information from current index...Lets update
                previousTimeStamp = out.gpsSV(epochCtr).gpsSec(prnCtr);
                prnCtr = prnCtr +1;

            end


    end
end